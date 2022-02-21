import sys
# path folder which contains situsim_v1_2
sys.path.insert(1, '..')
sys.path.insert(1, '../situsim_extensions')
from situsim_v1_2 import *
import pygame
import matplotlib.pyplot as plt
import time
from plots2 import *
from plots3 import *

# generate a circular arrangement of light sources
def sources_circle(n=20, r=9):
    sources = []
    for i in range(n):
        a = i * 2*np.pi / n
        sources.append(LightSource(r * np.cos(a), r * np.sin(a)))
    return sources

# compute average error over the robot's trajectory
def cost_function(robot):
    dists = np.sqrt(np.square(robot.xs) + np.square(robot.ys)) - 5 # 5 is the target circle radius
    return np.mean(np.abs(dists))

# A subclass of Controller
class OpenLoopCircleController(Controller):

    # init controller with passed in noisemakers and control parameters
    def __init__(self, left_noisemaker=None, right_noisemaker=None, speed=1, ratio=0.5):
        # NOTE: THIS CALL TO SUPER MUST BE HERE
        super().__init__(left_noisemaker, right_noisemaker) # call Controller.__init__() to set up noisemakers
        self.speed = speed
        self.ratio = ratio

    # step method. depending on the values of speed and ratio, the robot will drive along a circular path
    #   - but noise will be added to the control outputs, so the robot might not achieve its goal!
    def step(self, inputs, dt):

        # set left motor speed
        self.left_speed_command = self.speed
        # set right motor speed
        self.right_speed_command = self.speed * self.ratio

        return super().step(inputs, dt)

# A subclass of Controller
class ClosedLoopCircleController(Controller):

    # init controller with passed in noisemakers and control parameters
    def __init__(self, left_noisemaker=None, right_noisemaker=None, speed=1, ratio=0.5):
        # NOTE: THIS CALL TO SUPER MUST BE HERE
        super().__init__(left_noisemaker, right_noisemaker) # call Controller.__init__() to set up noisemakers
        self.speed = speed
        self.ratio = ratio

    # step method.
    # - MODIFY TO CLOSE THE SENSORIMOTOR LOOP!
    def step(self, inputs, dt):

        # set left motor speed
        self.left_speed_command = self.speed
        # set right motor speed
        self.right_speed_command = self.speed * self.ratio

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
                        noise_type='brown'
                        ):

    # get noisemakers for robot's motors
    left_motor_noisemaker = get_noisemaker(noise_type, left_motor_noise)
    right_motor_noisemaker = get_noisemaker(noise_type, right_motor_noise)

    # set up light sources
    light_sources = sources_circle(r=5)

    # construct the robot
    robot = Robot(x=-5, y=0, theta=np.pi/2,
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

    # our objective is to minimise this error
    print("Average error over time = " + str(cost_function(robot)))

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
    # plot_all_robots_controllers(all_ts, agents)
    # plot_all_robots_sensors(all_ts, agents)
    # plot_all_robots_motor_noise(all_ts, agents)
    # plot_all_robots_controller_noise(all_ts, agents)
    # plot_all_robots_sensor_noise(all_ts, agents)

    plt.show()

'''

select controller and parameters and run simulation

'''
def run_sim(runs=1, animate=True, left_motor_noise=0.1, right_motor_noise=0.1, noise_type='brown'):
    # set noise levels for controller outputs
    left_noise = 0.3
    right_noise = 0.4

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
        # - tweak speed and ratio to get robot to drive around circle
        # controller = OpenLoopCircleController(speed=0.5, ratio=0.665)

        # once you have open loop control working, use noise to make it
        # fail, and then program a closed loop controller so that the robot
        # can stay on the track again
        controller = ClosedLoopCircleController(speed=0.1, ratio=0.665)

        # if you uncomment this line, only the first run of the simulation will be animated
        animate = animate and i == 0

        # simulation duration
        duration = 150

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
                                                        noise_type=noise_type
                                                        )

        # build lists of robots and time values for plotting
        all_robots = all_robots + robots
        all_ts.append(ts)
    # plot data for all robots
    do_plots(all_ts, all_robots, light_sources)

# run the simulation
run_sim(runs=1, animate=True, left_motor_noise=0.01, right_motor_noise=0.01, noise_type='brown')
