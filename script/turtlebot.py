import pybullet as p
import pybullet_data
import time
from ball import *
from TurtleSFOC import *

bot_height = 0.4
ball_radius = 0.1
global_g = -10
simulation_step = 1./50.
previous_planning = True


def turtle_get_z0(turtle):
    '''
    Returns the x0, y0, and yaw0 of turtlebot.
    '''
    turtle_p_o = p.getBasePositionAndOrientation(turtle)
    turtle_position = turtle_p_o[0]
    # print(f'Turtle coordinate: {turtle_position}')
    turtle_euler = p.getEulerFromQuaternion(turtle_p_o[1])
    # print(f'Turtle Euler: {turtle_euler}')
    turtle_z0 = [turtle_position[0], turtle_position[1], turtle_euler[2]]
    # turtle z0 --> x, y, yaw
    print(f'Turtle Z0: {turtle_z0}')
    return turtle_z0


def get_ball_position(ball):
    '''
    Get the ball/desired position.
    '''
    ball_p_o = p.getBasePositionAndOrientation(ball)
    ball_position = ball_p_o[0]  # x y z coordinate of the ball
    ball_x = ball_position[0]
    ball_y = ball_position[1]
    ball_z = ball_position[2]
    return ball_x, ball_y, ball_z


def get_turtle_velocity(turtle):
    '''
Get the velocity of turtlebot's right and left wheels.
    '''
    turtle_left = p.getJointState(turtle, 0)[1]
    turtle_right = p.getJointState(turtle, 1)[1]
    turtle_pre = [turtle_right, turtle_left]
    return turtle_pre


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
offset = [0, 0, 0]
turtle = p.loadURDF("../model/turtlebot.urdf", offset, globalScaling=1.0)
plane = p.loadURDF("../model/plane.urdf")
# p.setRealTimeSimulation(1)
# create elastic ball
elastic_ball = create_elastic_ball(2, 1.5, 1)
# elastic collision with the floor plane
p.changeDynamics(plane, -1,
                 #  lateralFriction=0,
                 spinningFriction=0,
                 rollingFriction=0,
                 restitution=1)
# p.changeDynamics(turtle, -1, restitution = 0)


p.setGravity(0, 0, global_g)

for j in range(p.getNumJoints(turtle)):
    print(p.getJointInfo(turtle, j))
forward = 0
turn = 0
force_applied = False
# logId1 = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT, "LOG0003.txt")
while p.isConnected():
    print('-------------')
    # p.setRealTimeSimulation(1)
    # time.sleep(1.)
    # NEED TO USE STEP SIMULATION
    p.stepSimulation()
    p.setTimeStep(simulation_step)
    if not force_applied:
        p.applyExternalForce(
            elastic_ball, -1, [50, 50, 0], p.getBasePositionAndOrientation(elastic_ball)[0], flags=p.WORLD_FRAME)
        force_applied = True

    start_time = time.time()
    # ----- compute controls -----
    turtle_z0 = turtle_get_z0(turtle)
    # get ball states
    ball_x, ball_y, ball_z = get_ball_position(elastic_ball)
    ball_z = ball_z - ball_radius

    ball_vx, ball_vy, ball_vz = p.getBaseVelocity(elastic_ball)[0]

    turtle_pre = get_turtle_velocity(turtle)
    Time_limit = 2

    object_dist = np.sqrt(
        (turtle_z0[0] - (ball_x + ball_vx * Time_limit))**2 + (turtle_z0[1] - (ball_y + ball_vy * Time_limit))**2)
    turtle_velocity = np.sqrt(p.getBaseVelocity(turtle)[
        0][0] ** 2 + p.getBaseVelocity(turtle)[0][1] ** 2)
    print(f'Ball Z Position: {ball_z}')
    print(f'Ball Z Velocity: {ball_vz}')
    # if the ball is still relatively far away
    if object_dist >= 2:
        print('Approaching the desired position.')
        print(f'Ball distance: {object_dist}')
        feasible, output_right, output_left = Optimalcontrol_approach(
            turtle_z0, turtle_pre, ball_x + ball_vx * Time_limit, ball_y + ball_vy * Time_limit, Time_limit)
    else:
        print('Trying to catch to ball.')
        print(f'Ball distance: {object_dist}')
        # guess_period = (object_dist) / turtle_velocity
        guess_period = object_dist / (0.5 * turtle_velocity + 0.5 * 80 * 0.035)
        print(f'Guessed Period: {guess_period}.')
        if ball_vz > 0:
            MAX_HEIGHT = np.sqrt(ball_vz ** 2 / (-2*global_g)) + ball_z
            T_period = 2 * np.sqrt(2 * MAX_HEIGHT / -global_g)
        else:
            V_GROUND = np.sqrt(2 * global_g * (-ball_z) + ball_vz**2)
            T_period = 2 * V_GROUND / -global_g
        # 1. negative v, z < bot_z
        # 2. positive vm z < bot_z
        # 3. negative v, z > bot_z
        if ball_z > bot_height:
            T_lower = 0
            if ball_vz > 0:
                T_to_max = - ball_vz / global_g
                ball_z_max = ball_z + 0.5 * ball_vz * T_to_max
                T_upper = T_to_max + \
                    np.sqrt(2 * (ball_z_max - bot_height) / -global_g)
            else:
                v_same_height = np.sqrt(
                    2*global_g * (bot_height - ball_z) + ball_vz**2)
                T_upper = (-v_same_height - ball_vz) / global_g
        else:
            if ball_vz > 0:
                v_same_height = np.sqrt(
                    2 * global_g * (bot_height - ball_z) + ball_vz**2)
                T_lower = (v_same_height - ball_vz) / global_g

                T_to_max = -v_same_height / global_g
                ball_z_max = bot_height + 0.5 * v_same_height * T_to_max
                T_upper = 2 * T_to_max + T_lower
            else:
                v_ground = np.sqrt(2 * global_g * (-ball_z) + ball_vz**2)
                T_to_ground = (-v_ground - ball_vz) / global_g

                v_same_height = np.sqrt(
                    2 * global_g * (bot_height - ball_z) + ball_vz**2)
                T_lower = (v_same_height + ball_vz) / \
                    global_g + 2 * T_to_ground

                T_to_max = -v_same_height / global_g
                ball_z_max = bot_height + 0.5 * v_same_height * T_to_max
                T_upper = 2 * T_to_max + T_lower

        print(f'T_lower before: {T_lower}')
        print(f'T_upper before: {T_upper}')
        # feed current info into the feedback control algorithm
        while T_upper < guess_period and turtle_velocity > 0.2:
            T_upper = T_upper + T_period
        print(f'T_lower after: {T_lower}')
        print(f'T_upper after : {T_upper}')
        if T_upper > 0.5:
            feasible, output_right, output_left = Optimalcontrol(
                turtle_z0, turtle_pre, ball_x, ball_y, T_lower, T_upper, ball_vx, ball_vy)
            previous_planning = True
        else:
            time.sleep(0.2)
            if previous_planning == True:
                idx, time_passed = 0, 0
                previous_planning = False
            time_passed = time_passed + (simulation_step)
            idx = int(time_passed/0.05)
            if idx < len(output_left) - 1:
                output_left[0] = output_left[idx]
                output_right[0] = output_right[idx]
            else:
                output_left[0] = 0
                output_right[0] = 0

# ----- end of controls -----
    end_time = time.time()

    print(f'Feasible Control: {feasible}')
    print(f'Right Wheel Speed: {output_right[0]}')
    print(f'Left Wheel Speed: {output_left[0]}')
    print(f'Takes {end_time - start_time} seconds')

    # print(f"ball position: {p.getBasePositionAndOrientation(elastic_ball)[0]}")
    # apply wheel speeds from the control algorithm output
    p.setJointMotorControl2(turtle, 0, p.VELOCITY_CONTROL,
                            targetVelocity=output_left[0], force=1000)
    p.setJointMotorControl2(turtle, 1, p.VELOCITY_CONTROL,
                            targetVelocity=output_right[0], force=1000)