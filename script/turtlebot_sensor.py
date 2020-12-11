import pybullet as p
import pybullet_data
import time
from ball import *
from TurtleSFOC import *
import cv2
import numpy as np

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
    return ball_position


def get_turtle_velocity(turtle):
    '''                      
    Get the velocity of turtlebot's right and left wheels.
    '''
    turtle_left = p.getJointState(turtle, 0)[1]
    turtle_right = p.getJointState(turtle, 1)[1]
    turtle_pre = [turtle_right, turtle_left]
    return turtle_pre


def get_coord_from_cam(images):
    rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
    depth_buffer_opengl = np.reshape(images[3], [width, height])
    depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
    seg_opengl = np.reshape(images[4], [width, height]) * 1. / 255.
    img_coords = get_ball_coord(rgb_opengl[:, :, :3].astype(np.float32))
    est_coord = depth_opengl[img_coords[0], img_coords[1]] - 5
    return est_coord


def get_z_from_cam(images):
    rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
    depth_buffer_opengl = np.reshape(images[3], [width, height])
    depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
    seg_opengl = np.reshape(images[4], [width, height]) * 1. / 255.
    img_coords = get_ball_coord(rgb_opengl[:, :, :3].astype(np.float32))
    print(f'img_coords: {img_coords}')
    est_coord = 5 - depth_opengl[img_coords[0], img_coords[1]]
    return est_coord


def get_ball_coord(rgb_img):
    output_frame = rgb_img.copy()
    captured_frame_red = cv2.inRange(
        rgb_img*255, np.array([40, 0, 0]), np.array([255, 100, 100]))
    coords = np.mean(np.transpose(np.nonzero(captured_frame_red)), axis=0)
    cv2.imshow('frame', captured_frame_red)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # cv2.waitKey(1)
    return coords.astype(np.int32)


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
offset = [0, 0, 0]
turtle = p.loadURDF("../model/turtlebot.urdf", offset, globalScaling=1.0)
plane = p.loadURDF("../model/plane.urdf")
# p.setRealTimeSimulation(1)

width = 128
height = 128

fov = 60
aspect = width / height
near = 0.02
far = 10

dt = simulation_step
A = np.array([[1, dt], [0, 1]])
C = np.array([[1, 0]])
L = np.array([[1], [5]])
L_z = np.array([[1], [14]])

start_x, start_y, start_z = 1.5, 2, 1

x_estimation, y_estimation, z_estimation = np.array(
    [[start_x], [0]]), np.array([[start_y], [0]]), np.array([[start_z], [0]])


view_matrix_x = p.computeViewMatrix([-5, 0, 2.5], [0, 0, 2.5], [0, 0, 1])
view_matrix_y = p.computeViewMatrix([0, -5, 2.5], [0, 0, 2.5], [0, 0, 1])
view_matrix_z = p.computeViewMatrix([2.5, 2.5, 5], [2.5, 2.5, 0], [1, 0, 0])

projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

# create elastic ball
elastic_ball = create_elastic_ball(start_x, start_y, start_z)
# elastic_ball = create_elastic_ball(1, 3, 1)
# elastic collision with the floor plane
p.changeDynamics(plane, -1,
                 #  lateralFriction=0,
                 spinningFriction=0,
                 rollingFriction=0,
                 restitution=1)
# p.changeDynamics(turtle, -1, restitution = 0)


p.setGravity(0, 0, global_g)

prev_x, prev_y, prev_z = 0, 0, 0

for j in range(p.getNumJoints(turtle)):
    print(p.getJointInfo(turtle, j))
forward = 0
turn = 0
force_applied = False
# logId1 = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT, "forward_sensor.txt")
step = 0
true_x, true_y, true_z = [0], [0], [0]
vel_x, vel_y, vel_z = [0], [0], [0]
mean_x, mean_y = [0],[0]

while p.isConnected():
    # p.setRealTimeSimulation(1)
    # time.sleep(1.)
    # NEED TO USE STEP SIMULATION
    p.stepSimulation()
    p.setTimeStep(simulation_step)
    step = step + 1
    print(f'----- Simulation Step {step} -----')

    if not force_applied:
        p.applyExternalForce(
            elastic_ball, -1, [15, 7.5, 0], p.getBasePositionAndOrientation(elastic_ball)[0], flags=p.WORLD_FRAME)
        force_applied = True

    ball_x, ball_y, ball_z = get_ball_position(elastic_ball)

    print(f'True ball X: {ball_x}')
    print(f'True ball Y: {ball_y}')
    print(f'True ball Z: {ball_z}')

    # Get depth values using the OpenGL renderer
    images_x = p.getCameraImage(width,
                                height,
                                view_matrix_x,
                                projection_matrix,
                                shadow=True,
                                renderer=p.ER_BULLET_HARDWARE_OPENGL)
    images_y = p.getCameraImage(width,
                                height,
                                view_matrix_y,
                                projection_matrix,
                                shadow=True,
                                renderer=p.ER_BULLET_HARDWARE_OPENGL)
    images_z = p.getCameraImage(width,
                                height,
                                view_matrix_z,
                                projection_matrix,
                                shadow=True,
                                renderer=p.ER_BULLET_HARDWARE_OPENGL)
    # get estimated ball x coordinate
    sensor_x = get_coord_from_cam(images_x)
    sensor_y = get_coord_from_cam(images_y)
    sensor_z = get_z_from_cam(images_z)

    if step == 1:
        x_estimation, y_estimation, z_estimation = np.array(
            [[sensor_x], [0]]), np.array([[sensor_y], [0]]), np.array([[sensor_z], [0]])
    else:
        x_estimation = A @ x_estimation + L @ (sensor_x - C @ x_estimation)
        y_estimation = A @ y_estimation + L @ (sensor_y - C @ y_estimation)
        z_estimation = A @ z_estimation + L_z @ (sensor_z - C @ z_estimation) + np.array(
            [[-0.5 * global_g * 0.02**2], [-global_g * 0.02]])

    # dummy_vx = (estimate_x - prev_x) / 0.02
    # prev_x = dummy_vx

    # estimate_yv = (estimate_y - prev_y) / 0.02
    # prev_y = estimate_y

    # print(est_x, ball_x)

    start_time = time.time()
    # ----- compute controls -----
    turtle_z0 = turtle_get_z0(turtle)
    # get ball states
    # ball_x, ball_y, ball_z = get_ball_position(elastic_ball)
    ball_x, ball_y, ball_z = x_estimation[0,
                                          0], y_estimation[0, 0], z_estimation[0, 0]
    ball_z = ball_z - ball_radius

    ball_vx, ball_vy, ball_vz = p.getBaseVelocity(elastic_ball)[0]

    true_x.append(ball_vx)
    true_y.append(ball_vy)
    true_z.append(ball_vz)
    dummy_x, dummy_y, dummy_z = x_estimation[1,
                                             0], y_estimation[1, 0], z_estimation[1, 0]

    vel_x.append(dummy_x)
    vel_y.append(dummy_y)
    vel_z.append(dummy_z)

    if len(vel_x) < 10:
        mean_x.append(np.mean(vel_x))
        mean_y.append(np.mean(vel_x))
    else:
        mean_x.append(np.mean(vel_x[-10:]))
        mean_y.append(np.mean(vel_y[-10:]))

    

    print(f'Sensor ball X: {sensor_x}')
    print(f'Sensor ball Y: {sensor_y}')
    print(f'Sensor ball Z: {sensor_z}')

    print(f'Estimated ball X: {ball_x}')
    print(f'Estimated ball Y: {ball_y}')
    print(f'Estimated ball Z: {ball_z}')

    print('<------>')

    print(f'Ball X Velocity: {ball_vx}')
    print(f'Estimate X Velocity: {dummy_x}')
    print(f'Ball Y Velocity: {ball_vy}')
    print(f'Estimate Y Velocity: {dummy_y}')
    print(f'Ball Z Velocity: {ball_vz}')
    print(f'Estimate Z Velocity: {dummy_z}')

    ball_vx, ball_vy, ball_vz = mean_x[-1], mean_y[-1], dummy_z - 1
    turtle_pre = get_turtle_velocity(turtle)
    Time_limit = 2

    object_dist = np.sqrt(
        (turtle_z0[0] - (ball_x + ball_vx * Time_limit))**2 + (turtle_z0[1] - (ball_y + ball_vy * Time_limit))**2)
    turtle_velocity = np.sqrt(p.getBaseVelocity(turtle)[
        0][0] ** 2 + p.getBaseVelocity(turtle)[0][1] ** 2)
    # print(f'Ball Z Position: {ball_z}')
    # print(f'Ball Z Velocity: {ball_vz}')
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
        # while T_upper < guess_period and turtle_velocity > 0.2:
        #     T_upper = T_upper + T_period
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
    
    # if step == 100:
    #     plt.figure(1)
    #     plt.plot(true_x, 'b')
    #     plt.plot(vel_x, 'r')
    #     plt.plot(mean_x, 'g')
    #     plt.figure(2)
    #     plt.plot(true_y, 'b')
    #     plt.plot(vel_y, 'r')
    #     plt.plot(mean_y, 'g')
    #     plt.figure(3)
    #     plt.plot(true_z, 'b')
    #     plt.plot([z - 1 for z in vel_z], 'g')
    #     plt.plot(vel_z, 'r')
    #     plt.legend()
    #     plt.show()
    #     continue
