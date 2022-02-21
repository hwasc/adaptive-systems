import sys
# path folder which contains situsim_v1_2
sys.path.insert(1, '..')
sys.path.insert(1, '../situsim_extensions')
from situsim_v1_2 import *
import pygame
import matplotlib.pyplot as plt
import time
from plots2 import *

# A subclass of Controller, which implements Braitenberg's coward
class CowardController(Controller):

    # init controller with passed in noisemakers and control parameters
    def __init__(self, left_noisemaker, right_noisemaker, gain=1):
        # NOTE: THIS CALL TO SUPER MUST BE HERE FOR NOISYCONTROLLERS!
        super().__init__(left_noisemaker, right_noisemaker) # call NoisyController.__init__() to set up noisemakers
        self.gain = gain

    # step method. depending on the values of speed and ratio, the robot will drive along a circular path
    #   - but noise will be added to the control outputs, so the robot might not achieve its goal!
    def step(self, inputs, dt):

        '''
            Set motor speeds, based on sensory input.

            inputs[0] is left sensor output
            inputs[1] is right sensor output

            self.left_speed_command is the command which will be sent to the left motor
            self.right_speed_command is the command which will be sent to the right motor

            the speed commands will typically be functions of the inputs
            if the speed commands are not functions of the inputs, then this is an open loop controller

            the mapping from inputs to commands can take any form, e.g.:
                simple equations, as in Braitenberg vehicle implementations
                less simple equations, which might combine equations with logic
                neural networks, feedforward or dynamical
                homeostats (more on this later)
                other dynamical systems

        '''

        # set left motor speed
        self.left_speed_command = self.gain * inputs[0]
        # set right motor speed
        self.right_speed_command = self.gain * inputs[1]

        return super().step(inputs, dt)

# A subclass of Controller, which implements Braitenberg's aggressor
class AggressorController(Controller):

    # init controller with passed in noisemakers and control parameters
    def __init__(self, left_noisemaker, right_noisemaker, gain=1):
        # NOTE: THIS CALL TO SUPER MUST BE HERE FOR NOISYCONTROLLERS!
        super().__init__(left_noisemaker, right_noisemaker) # call NoisyController.__init__() to set up noisemakers
        self.gain = gain

    # step method. depending on the values of speed and ratio, the robot will drive along a circular path
    #   - but noise will be added to the control outputs, so the robot might not achieve its goal!
    def step(self, inputs, dt):

        '''
            Set motor speeds, based on sensory input.

            inputs[0] is left sensor output
            inputs[1] is right sensor output

            self.left_speed_command is the command which will be sent to the left motor
            self.right_speed_command is the command which will be sent to the right motor

            the speed commands will typically be functions of the inputs
            if the speed commands are not functions of the inputs, then this is an open loop controller

            the mapping from inputs to commands can take any form, e.g.:
                simple equations, as in Braitenberg vehicle implementations
                less simple equations, which might combine equations with logic
                neural networks, feedforward or dynamical
                homeostats (more on this later)
                other dynamical systems

        '''

        # set left motor speed
        self.left_speed_command = self.gain * inputs[1]
        # set right motor speed
        self.right_speed_command = self.gain * inputs[0]

        return super().step(inputs, dt)

# A subclass of Controller, which implements Braitenberg's lover
class LoverController(Controller):

    # init controller with passed in noisemakers and control parameters
    def __init__(self, left_noisemaker, right_noisemaker, gain=1, bias=0.15):
        # NOTE: THIS CALL TO SUPER MUST BE HERE FOR NOISYCONTROLLERS!
        super().__init__(left_noisemaker, right_noisemaker) # call NoisyController.__init__() to set up noisemakers
        self.gain = gain
        self.bias = bias

    # step method. depending on the values of speed and ratio, the robot will drive along a circular path
    #   - but noise will be added to the control outputs, so the robot might not achieve its goal!
    def step(self, inputs, dt):

        '''

            Set motor speeds, based on sensory input.

            inputs[0] is left sensor output
            inputs[1] is right sensor output

            self.left_speed_command is the command which will be sent to the left motor
            self.right_speed_command is the command which will be sent to the right motor

            the speed commands will typically be functions of the inputs
            if the speed commands are not functions of the inputs, then this is an open loop controller

            the mapping from inputs to commands can take any form, e.g.:
                simple equations, as in Braitenberg vehicle implementations
                less simple equations, which might combine equations with logic
                neural networks, feedforward or dynamical
                homeostats (more on this later)
                other dynamical systems

        '''

        # set left motor speed
        self.left_speed_command = -self.gain * inputs[0]
        # set right motor speed
        self.right_speed_command = -self.gain * inputs[1]


        return super().step(inputs, dt)

# A subclass of Controller, which implements Braitenberg's lover
class MonocularController(Controller):

    # init controller with passed in noisemakers and control parameters
    def __init__(self, left_noisemaker, right_noisemaker, gain=1, bias=0.15):
        # NOTE: THIS CALL TO SUPER MUST BE HERE FOR NOISYCONTROLLERS!
        super().__init__(left_noisemaker, right_noisemaker) # call NoisyController.__init__() to set up noisemakers
        self.gain = gain
        self.bias = bias

    # step method. depending on the values of speed and ratio, the robot will drive along a circular path
    #   - but noise will be added to the control outputs, so the robot might not achieve its goal!
    def step(self, inputs, dt):

        '''

            Set motor speeds, based on sensory input.

            inputs[0] is left sensor output
            inputs[1] is right sensor output

            self.left_speed_command is the command which will be sent to the left motor
            self.right_speed_command is the command which will be sent to the right motor

            the speed commands will typically be functions of the inputs
            if the speed commands are not functions of the inputs, then this is an open loop controller

            the mapping from inputs to commands can take any form, e.g.:
                simple equations, as in Braitenberg vehicle implementations
                less simple equations, which might combine equations with logic
                neural networks, feedforward or dynamical
                homeostats (more on this later)
                other dynamical systems

        '''
        # print(inputs[0])
        if  inputs[0] < 0.016:
            self.left_speed_command = self.gain * 0.1
            self.right_speed_command = 0
        else:
            self.left_speed_command = 0
            self.right_speed_command = self.gain * 0.1

        return super().step(inputs, dt)

# set up the pygame window, if we are animating the simulation
def setup_pygame_window(screen_width):
    # initialise pygame and set parameters
    pygame.init()
    screen = pygame.display.set_mode([screen_width, screen_width])
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

# main function, to run simulation and generate plots
def run_simulation_once(screen_width, controller, animate=False,
                        field_of_view=0.9*np.pi,
                        left_sensor_angle=np.pi/4,
                        right_sensor_angle=-np.pi/4,
                        duration=60,
                        radius=5):

    # set up light sources
    light_sources = [LightSource(x=0, y=0, brightness=2)]

    # randomise robot position and orientation
    x = random_in_interval(minimum=-radius, maximum=radius)
    y = random_in_interval(minimum=-radius, maximum=radius)
    theta = random_in_interval(minimum=-np.pi, maximum=np.pi)

    # construct the robot
    robot = Robot(x=x, y=y, theta=theta,
                  controller=controller,
                  field_of_view=field_of_view,
                  left_light_sources=light_sources,
                  right_light_sources=light_sources,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=100,
                  right_motor_inertia=100
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

    return ts, agents, light_sources

# plot outputs for all robots
# - note: these are not all of the outputs which we can plot,
# but they are the ones which will always be available,
# and which we will probably look at the most
def do_plots(all_ts, agents, light_sources):

    # parameters for plots
    plt.rcParams["font.weight"] = "bold"
    font_size = 18

    plot_all_agents_trajectories(all_ts, agents, light_sources)
    # plot_all_robots_motors(all_ts, agents)
    # plot_all_robots_controllers(all_ts, agents)
    # plot_all_robots_sensors(all_ts, agents)

    plt.show()

'''

select controller and parameters and run simulation

'''
def run_sim(runs=1, animate=True):
    # set noise levels for controller outputs
    left_noise = 0
    right_noise = 0

    # make noisemakers None by default, although they may get set later
    left_noisemaker = None
    right_noisemaker = None

    all_robots = []
    all_ts = []

    # run the simulation the specified number of times
    for i in range(runs):

        # NOTE: ALL SYSTEMS ARE CREATED INSIDE THIS LOOP, OR IN FUNCTIONS CALLED BY IT
        # - if we created a system outside of this loop, e.g. one of the noisemakers,
        # then it would be used in every run

        # there are other kinds of NoiseSource available to try
        if left_noise > 0:
            left_noisemaker = BrownNoiseSource(left_noise)
        if right_noise > 0:
            right_noisemaker = BrownNoiseSource(right_noise)

        # # there are other kinds of NoiseSource available to try
        # if left_noise > 0:
        #     left_noisemaker = WhiteNoiseSource(min_val=0, max_val=left_noise)
        # if right_noise > 0:
        #     right_noisemaker = WhiteNoiseSource(min_val=0, max_val=left_noise)

        # I USED THESE SENSOR MORPHOLOGY PARAMETERS FOR THE ALL CONTROLLERS EXCEPT THE LOVER
        field_of_view = 0.9*np.pi
        left_sensor_angle = np.pi/4
        right_sensor_angle = -np.pi/4

        # # create a controller object to pass to the robot
        # controller = CowardController(left_noisemaker=left_noisemaker,
        #                               right_noisemaker=right_noisemaker,
        #                               gain=2)

        # # create a controller object to pass to the robot
        # controller = AggressorController(left_noisemaker=left_noisemaker,
        #                               right_noisemaker=right_noisemaker,
        #                               gain=2)

        controller = MonocularController(left_noisemaker=left_noisemaker,
                                    right_noisemaker=right_noisemaker,
                                    gain=2, bias=0.2)

        # I USED THESE SENSOR MORPHOLOGY PARAMETERS FOR THE LOVER
        # field_of_view=1.4*np.pi
        # left_sensor_angle = np.pi/4
        # right_sensor_angle = -np.pi/4
        
        # # create a controller object to pass to the robot
        # controller = LoverController(left_noisemaker=left_noisemaker,
        #                             right_noisemaker=right_noisemaker,
        #                             gain=2, bias=0.15)

        # if you uncomment this line, only the first run of the simulation will be animated
        animate = animate and i == 0

        # duration = 60
        duration = 240 # longer for monocular controller

        # sets how far the robots will initially be from light
        radius =10 # 5

        # run the simulation once, with the given parameters
        ts, robots, light_sources = run_simulation_once(screen_width=1800,
                                                    controller=controller,
                                                    animate=animate,
                                                    field_of_view=field_of_view,
                                                    left_sensor_angle=left_sensor_angle,
                                                    right_sensor_angle=right_sensor_angle,
                                                    duration=duration,
                                                    radius=radius
                                                    )

        all_robots = all_robots + robots
        all_ts.append(ts)

    do_plots(all_ts, all_robots, light_sources)

# run_sim(30, animate=False)

run_sim(10, animate=True)
