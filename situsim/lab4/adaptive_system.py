import sys

from numpy import float64
# path folder which contains situsim_v1_2
sys.path.insert(1, '..')
sys.path.insert(1, '../situsim_extensions')
from situsim_v1_2 import *
import pygame
import matplotlib.pyplot as plt
import time
import copy

from plots2 import *
from disturbances import *
import random

import neat
# import neat.visualize as visualize
import os

# A subclass of Controller, which implements Braitenberg's aggressor (i.e. lightseeker)
class NeatController(Controller):

    # init controller with passed in noisemakers and control parameters
    def __init__(self, x, y, network, left_noisemaker=None, right_noisemaker=None, gain=1):
        # NOTE: THIS CALL TO SUPER MUST BE HERE FOR NOISYCONTROLLERS!
        super().__init__(left_noisemaker, right_noisemaker) # call NoisyController.__init__() to set up noisemakers
        self.gain = gain
        self.network = network
        self.start_x = x
        self.start_y = y
        self.target_x = 0
        self.target_y = 0
        self.prev_x = x
        self.prev_y = y
        self.score = 0
        self.total_distance = 0
        self.steps = 0
        self.time_at_target = 0
        self.inputs += [[0, 0]]
        self.right_speed_commands += [0]
        self.left_speed_commands += [0]

    
    def get_score(self):
        min_distance = np.sqrt((self.start_x - self.target_x)**2 + (self.start_y - self.target_y)**2)
        return self.score  #-  abs(min_distance - self.total_distance)/abs(min_distance)
        # return self.time_at_target
    
    def update_score(self, x, y):
        prev_dist = (self.prev_x - self.target_x)**2 + (self.prev_y - self.target_y)**2
        current_dist = (x - self.target_x)**2 + (y - self.target_y)**2
        if prev_dist > current_dist:
            self.score += 1*abs(prev_dist - current_dist)
        
        if prev_dist < current_dist:
            self.score -= 1*abs(prev_dist - current_dist)
        
        if current_dist < 2:
            self.time_at_target += 1
        
        self.steps += 1

        self.total_distance += abs(self.prev_x - x) + abs(self.prev_y - y)
        self.prev_x = x
        self.prev_y = y


    # step method. depending on the values of speed and ratio, the robot will drive along a circular path
    #   - but noise will be added to the control outputs, so the robot might not achieve its goal!
    def step(self, inputs, dt):
        input_left = np.array(self.inputs)[-2:, 0]
        input_right = np.array(self.inputs)[-2:, 1]

        prev_output_left = np.array(self.left_speed_commands)[-2:]
        prev_output_right = np.array(self.right_speed_commands)[-2:]

        input = np.array([input_left, input_right, prev_output_left, prev_output_right]).flatten()
        # print(input)
        output = self.network.activate(input)

        in_left = inputs[0]
        in_right = inputs[1]

        

        #swith inputs
        if output[0] > 0.5:
            in_left, in_right = in_right, in_left

        #spin
        # if output[1] > 0.5:
        if in_left < 0.001 and in_right < 0.001:
            in_left = 1
            in_right = -1

        # set left motor speed
        self.left_speed_command = self.gain * in_left
        # set right motor speed
        self.right_speed_command = self.gain * in_right
    
        return super().step(inputs, dt)


# set up the pygame window, if we are animating the simulation
def setup_pygame_window(screen_width):
    # initialise pygame and set parameters
    pygame.init()
    screen = pygame.display.set_mode([screen_width, screen_width])
    pygame.display.set_caption("SituSim: sensory inversion challenge")
    # scale factor and offsets for converting simulation coordinates to pygame animation display coordinates
    pygame_scale = 30
    pygame_x_offset = screen_width/2
    pygame_y_offset = screen_width/2

    return screen

# draw SituSim systems in pygame window
def pygame_drawsim(screen, systems, width, paused, delay):

    running = True

    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                paused = not paused
            elif event.key == pygame.K_UP:
                delay -= 1
            elif event.key == pygame.K_DOWN:
                delay += 1

    delay = np.max([delay, 0])

    time.sleep(delay/100)

    screen.fill('black')

    # initial scale factor and offsets for converting simulation coordinates
    # to pygame animation display coordinates
    pygame_x_offset = width/2
    pygame_y_offset = width/2

    # find extremes of system trajectories for resizing animation window
    max_xs = []
    max_ys = []
    for system in systems:
        if system.has_position:
            max_xs.append(max(np.abs(system.xs)))
            max_ys.append(max(np.abs(system.ys)))

    # reset scale according to where systems are and have been
    pygame_scale = width / (2 * max(max(max_xs), max(max_ys)) + 1)

    # draw all systems
    for system in systems:
        system.pygame_draw(screen, scale=pygame_scale, shiftx=pygame_x_offset, shifty=pygame_y_offset)

    # flip the pygame display
    screen.blit(pygame.transform.flip(screen, False, True), (0, 0))
    # update the pygame display
    pygame.display.update()

    return running, paused, delay

# get a noisemaker. used here for robot's motors, but could also be used for its
# sensors or controllers. this is only for convenience - for all except the
# BrownNoiseSource, it would be better to have more control over the parameters
def get_noisemaker(noise_type, noise_param):
    noisemaker = None
    if noise_param > 0:
        if noise_type == 'brown':
            noisemaker = BrownNoiseSource(noise_param)
        elif noise_type == 'white':
            noisemaker = WhiteNoiseSource(min_val=-noise_param, max_val=noise_param)
        elif noise_type == 'spike':
            noisemaker = SpikeNoiseSource(prob=0.05, pos_size=noise_param, neg_size=-noise_param)
    return noisemaker

# main function, to run simulation and generate plots
def run_sim_once(screen_width,
                    robots,
                    light_sources,
                    disturbances=None,
                    animate=False,
                    duration=60,
                ):

    # only run pygame code if animating the simulation
    if animate:
        screen = setup_pygame_window(screen_width)

    # animation variables
    delay = 0 # can be used to slow animation down
    running = True # can be used to exit animation early
    paused = False # can be used to pause simulation/animation

    # prepare simulation time variables
    t = 0
    ts = [t]
    dt = 0.1
    # begin simulation main loop
    while t < duration and running:

        # only move simulation forwards in time if not paused
        if not paused:
            # step disturbance
            for disturbance in disturbances:
                if disturbance is not None:
                    disturbance.step(dt)

            # step all robots
            for robot in robots:
                robot.step(dt)
                robot.controller.update_score(robot.x, robot.y)

            # increment time variable and store in ts list for plotting later
            t += dt
            ts.append(t)

        # only run pygame code if animating the simulation
        if animate:
            running, paused, delay = pygame_drawsim(screen, robots + light_sources, screen_width, paused, delay)
    # simulation has completed

    # only run pygame code if animating the simulation
    # if animate:
        # Quit pygame.
        # pygame.display.quit()
        # pygame.quit()

    return ts, robots, light_sources


# # run the simulation
# run_sim(runs=500, animate=True, disturb_times=[],
#         left_motor_noise=0.0, right_motor_noise=0.0, noise_type='brown')

def init_robot(network, light_sources):
    field_of_view = 0.9*np.pi
    left_sensor_angle = np.pi/4
    right_sensor_angle = -np.pi/4

    # get noisemakers for robot's motors
    noise_type = "brown"
    left_motor_noise = 0.0
    right_motor_noise = 0.0
    left_motor_noisemaker = get_noisemaker(noise_type, left_motor_noise)
    right_motor_noisemaker = get_noisemaker(noise_type, right_motor_noise)


    allowed_values = list(range(-30, 30))#list(range(-30, -27)) + list(range(27, 30))
    x = random_in_interval(minimum=-20, maximum=20)#random.choice(allowed_values)
    y = random_in_interval(minimum=-20, maximum=20)#random.choice(allowed_values)
    theta = random_in_interval(-np.pi/2, np.pi/2)  # at least one sensor should always have the light in view from here

    controller = NeatController(x, y, network=network, gain=20)
    
    # construct the robot
    robot = Robot(x=x, y=y, theta=theta,
                    controller=controller,
                    field_of_view=field_of_view,
                    left_light_sources=light_sources,
                    right_light_sources=light_sources,
                    left_sensor_angle=left_sensor_angle,
                    right_sensor_angle=right_sensor_angle,
                    left_motor_inertia=0,
                    right_motor_inertia=0,
                    left_motor_noisemaker=left_motor_noisemaker,
                    right_motor_noisemaker=right_motor_noisemaker
                )
    
    return robot

animate = False

def eval_genomes(genomes, config):
    global animate

    duration = random.randrange(50, 100)
    # duration = 50
    disturb_times = []
    for i in range(random.randint(1, 4)):
        disturb_times.append(random.randrange(1, 100))
    disturb_times = sorted(disturb_times)

    light_sources = [LightSource(x=0, y=0, brightness=2)]
    robots = []
    disturbances = []

    for genome_id, genome in genomes:
        network = neat.nn.FeedForwardNetwork.create(genome, config)
        robot = init_robot(network, light_sources)
        robots.append(robot)
    
        disturb = bool(disturb_times)
        
        if disturb:
            disturbances.append(SensoryInversionDisturbanceSource(robot, start_times=copy.deepcopy(disturb_times)))

    # t = 0
    # while t < duration:
    # run the simulation once, with the given parameters
    ts, robots, light_sources = run_sim_once(screen_width=700,
                                                robots=robots,
                                                light_sources=light_sources,
                                                disturbances=disturbances,
                                                animate=animate,
                                                duration=duration,
                                            )
    
    for genome, robot in zip(genomes, robots):
        # print(robot.controller)
        # robot.controller.score(robot.x, robot.y)
        genome[1].fitness = robot.controller.get_score()
        print(robot.controller.get_score())


def run(config_file):
    # Load configuration.
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_file)

    # Create the population, which is the top-level object for a NEAT run.
    p = neat.Population(config)

    global animate
    # animate = True 
    # p = neat.Checkpointer.restore_checkpoint('neat-checkpoint-44')

    # Add a stdout reporter to show progress in the terminal.
    p.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    p.add_reporter(stats)
    p.add_reporter(neat.Checkpointer(50))

    # # Run for up to 300 generations.
    winner = p.run(eval_genomes, 50)

    # # Display the winning genome.
    # print('\nBest genome:\n{!s}'.format(winner))

    # global animate
    animate = True
    # p.run(eval_genomes, 1)
    for i in range(30):
        eval_genomes([(1, winner)], config)


if __name__ == '__main__':
    # Determine path to configuration file. This path manipulation is
    # here so that the script will run successfully regardless of the
    # current working directory.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'adaptive_config_ff')
    run(config_path)