import sys
# path folder which contains situsim_v1_2
sys.path.insert(1, '..')
sys.path.insert(1, '../situsim_extensions')
from situsim_v1_2 import *
import pygame
import matplotlib.pyplot as plt
import time

from plots2 import *
from disturbances import *

# A subclass of Controller, which implements Braitenberg's aggressor (i.e. lightseeker)
class AdaptiveController(Controller):

    # init controller with passed in noisemakers and control parameters
    def __init__(self, left_noisemaker=None, right_noisemaker=None, gain=1):
        # NOTE: THIS CALL TO SUPER MUST BE HERE FOR NOISYCONTROLLERS!
        super().__init__(left_noisemaker, right_noisemaker) # call NoisyController.__init__() to set up noisemakers
        self.gain = gain
        self.event_detected = False
        self.reverse = False
        self.outputs_true = [[0,0]]

    # step method. depending on the values of speed and ratio, the robot will drive along a circular path
    #   - but noise will be added to the control outputs, so the robot might not achieve its goal!
    def step(self, inputs, dt):
        # if len(self.inputs) > 5:
        #     d_input_left = np.array(self.inputs)[-2:,0]
        #     d_input_right = np.array(self.inputs)[-2:,1]
        #     d_input = d_input_left - d_input_right

        #     d_output_left = np.array(self.left_speed_commands)[-2:]
        #     d_output_right = np.array(self.right_speed_commands)[-2:]
        #     d_output = d_output_left - d_output_right

        #     print("left: ", d_input_left[0], d_input_left[1], np.sign(d_input_left[0] - d_input_left[1]))
        #     print("right: ", d_input_right[0], d_input_right[1], np.sign(d_input_right[0] - d_input_right[1]))


            
        #     if d_input_left[0] < d_input_right[0]:
        #         # set left motor speed
        #         self.left_speed_command += 0.1
        #         # set right motor speed
        #         self.right_speed_command = self.gain * inputs[1]
        #         # if d_input_left[0] < d_input_left[1]:
        #         #     print("Left In: Good")
        #         #     self.reverse = False
        #         # elif d_input_right[0] < d_input_right[1]:
        #         #     print("Left In: Good")
        #         #     self.reverse = False
        #         # else:
        #         #     print("Left In: Bad")
        #         #     self.reverse = True
                
        #         # if d_output[1] - d_output[0] < 0:
        #         #     print("Right Out: Good")
        #         #     self.reverse = False
        #         # else:
        #         #     print("Right Out: Bad")
        #         #     self.reverse = True
        #     elif d_input_right[0] < d_input_left[0]:
        #         # set left motor speed
        #         self.left_speed_command = self.gain * inputs[1]
        #         # set right motor speed
        #         self.right_speed_command += 0.1
        #         # if d_input_right[0] < d_input_right[1]:
        #         #     print("Right In: Good")
        #         #     self.reverse = False
        #         # elif d_input_left[0] < d_input_left[1]:
        #         #     print("Right In: Good")
        #         #     self.reverse = False
        #         # else:
        #         #     print("Right In: Bad")
        #         #     self.reverse = True
        #         # if d_output[1] - d_output[0] < 0:
        #         #     print("Left Out: Good")
        #         #     self.reverse = False
        #         # else:
        #         #     print("Left Out: Bad")
        #         #     self.reverse = True
        #     else:
        #         # set left motor speed
        #         self.left_speed_command = self.gain * inputs[1]
        #         # set right motor speed
        #         self.right_speed_command = self.gain * inputs[1]
        
        if np.allclose(self.inputs[-1],[0.00000000, 0.00000000]):
            # set left motor speed
            self.left_speed_command = -0.1
            # set right motor speed
            self.right_speed_command = -0.1
        else:
            # set left motor speed
            self.left_speed_command = 0.1
            # set right motor speed
            self.right_speed_command = -0.1
        
        print(self.inputs[-1])

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
def run_simulation_once(screen_width,
                        controller,
                        animate=False,
                        field_of_view=0.9*np.pi,
                        left_sensor_angle=np.pi/4,
                        right_sensor_angle=-np.pi/4,
                        duration=60,
                        left_motor_noise=0.1,
                        right_motor_noise=0.1,
                        noise_type='brown',
                        disturb_times=[]
                        ):

    # get noisemakers for robot's motors
    left_motor_noisemaker = get_noisemaker(noise_type, left_motor_noise)
    right_motor_noisemaker = get_noisemaker(noise_type, right_motor_noise)

    # set up light sources
    light_sources = [LightSource(x=0, y=0, brightness=2)]

    # robots are always started from a position and orientation where they can see the light
    # it might be better if they started from all sides of the light, but this is slightly easier and roughly the
    # same
    x = -10
    y = random_in_interval(-7, 7)
    theta = random_in_interval(-np.pi/2, np.pi/2)  # at least one sensor should always have the light in view from here

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

    # unlike some other disturbances, the SensoryInversionDisturbanceSource is a one-shot disturbance;
    # it doesn't happen repeatedly in a specified interval. for this reason, it is only passed start_times, and no
    # end_times
    # at each start_time, the connections between sensors and motors are effectively swapped, turning a
    # light-seeking robot to a light-avoiding one, or vice versa
    disturb = bool(disturb_times) # only disturb if the times list is not empty
    if disturb:
        disturbance = SensoryInversionDisturbanceSource(robot, start_times=disturb_times) # disturb once, half-way through


    # create list of agents - even though we only have one here, I always code
    # using a list, as it makes it easy to add more agents
    agents = [robot]

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
            print("time: ", t)

            # step disturbance
            if disturb:
                disturbance.step(dt)

            # step all robots
            for agent in agents:
                agent.step(dt)

            # increment time variable and store in ts list for plotting later
            t += dt
            ts.append(t)

        # only run pygame code if animating the simulation
        if animate:
            running, paused, delay = pygame_drawsim(screen, agents + light_sources, screen_width, paused, delay)
    # simulation has completed

    # only run pygame code if animating the simulation
    if animate:
        # Quit pygame.
        pygame.display.quit()
        pygame.quit()

    return ts, agents, light_sources

# plot outputs for all robots
# - note: these are not all of the outputs which we can plot,
# but they are the ones which will always be available,
# and which we will probably look at the most
def do_plots(all_ts, agents, light_sources):

    # parameters for plots
    plt.rcParams["font.weight"] = "bold"
    font_size = 18

    plot_all_agents_trajectories(all_ts, agents, light_sources, draw_agents=False)
    # plot_all_robots_motors(all_ts, agents)

    plt.show()

'''

select controller and parameters and run simulation

'''
def run_sim(runs=1, animate=True, disturb_times=[], left_motor_noise=0.1, right_motor_noise=0.1, noise_type='brown'):
    # set noise levels for controller outputs
    left_noise = 0
    right_noise = 0

    all_robots = []
    all_ts = []

    # run the simulation the specified number of times
    for i in range(runs):

        # NOTE: ALL SYSTEMS ARE CREATED INSIDE THIS LOOP, OR IN FUNCTIONS CALLED BY IT
        # - if we created a system outside of this loop, e.g. one of the noisemakers,
        # then it would be used in every run

        field_of_view = 0.9*np.pi
        left_sensor_angle = np.pi/4
        right_sensor_angle = -np.pi/4

        # create a controller object to pass to the robot
        controller = AdaptiveController(gain=2)

        # if you uncomment this line, only the first run of the simulation will be animated
        animate = animate and i == 0

        duration = 400 # longer for monocular controller

        # use a copy of the disturb_times for every run
        # - this is required due to what *seemed like* a good idea when I first
        #   programmed the DisturbanceSource class - times get popped from the lists
        #   as they are used, meaning the lists can only be used once (doh!)
        disturb_times2 = []
        for t in disturb_times:
            disturb_times2.append(t)

        # run the simulation once, with the given parameters
        ts, robots, light_sources = run_simulation_once(screen_width=700,
                                                        controller=controller,
                                                        animate=animate,
                                                        field_of_view=field_of_view,
                                                        left_sensor_angle=left_sensor_angle,
                                                        right_sensor_angle=right_sensor_angle,
                                                        duration=duration,
                                                        left_motor_noise=left_motor_noise,
                                                        right_motor_noise=right_motor_noise,
                                                        noise_type=noise_type,
                                                        disturb_times=disturb_times2
                                                        )

        d_input_left = np.array(controller.inputs)[:,0]
        d_input_right = np.array(controller.inputs)[:,1]
        d_input = d_input_left - d_input_right

        N = 10
        d_input = np.convolve(d_input, np.ones(N)/N, mode='same')

        d_output_left = np.array(controller.left_speed_commands)[:]
        d_output_right = np.array(controller.right_speed_commands)[:]
        d_output = d_output_left - d_output_right

        d_output = np.convolve(d_output, np.ones(N)/N, mode='same')
        
        # din1 = np.diff(d_input)
        # din2 = np.diff(din1)
        # dout1 = np.diff(d_output)
        # dout2 = np.diff(dout1)
        # print(din1, din2, np.sum(np.sign(din2)))
        # print(dout1, dout2, np.sum(np.sign(dout2)))

        # plt.plot(np.linspace(0, 200, len(din1)), din1)#d_input_left*d_output_right)
        # plt.plot(np.linspace(0, 200, len(d_input_left)), d_input_left, color='orange')
        # plt.plot(np.linspace(0, 200, len(d_input_left)), d_output_left, color='r')

        # plt.plot(np.linspace(0, 200, len(d_input_left)), d_input_right, color='g')
        # plt.plot(np.linspace(0, 200, len(d_input_left)), d_output_right, color='b')
        # plt.show()

        all_robots = all_robots + robots
        all_ts.append(ts)

    do_plots(all_ts, all_robots, light_sources)

# run the simulation
run_sim(runs=10, animate=True, disturb_times=[],
        left_motor_noise=0.0, right_motor_noise=0.0, noise_type='brown')
