import pybullet as p
import pybullet_data
import time
from ball import *
from TurtleSFOC import *

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
	ball_position = ball_p_o[0] # x y z coordinate of the ball 
	ball_x = ball_position[0]
	ball_y = ball_position[1]
	return ball_x, ball_y

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
offset = [0,0,0]
turtle = p.loadURDF("../model/turtlebot.urdf",offset)
plane = p.loadURDF("plane.urdf")
# p.setRealTimeSimulation(1)
# create elastic ball
elastic_ball = create_elastic_ball(1, 1, 6)
# elastic collision with the floor plane
p.changeDynamics(plane, -1, restitution=1)

p.setGravity(0,0,-10)

for j in range (p.getNumJoints(turtle)):
	print(p.getJointInfo(turtle,j))
forward=0
turn=0
while (1):
	# p.setRealTimeSimulation(1)
	# time.sleep(1./24.)
	# NEED TO USE STEP SIMULATION
	p.stepSimulation()
	p.setTimeStep(1./50.)

	turtle_z0 = turtle_get_z0(turtle)
	ball_x, ball_y = get_ball_position(elastic_ball)
	turtle_pre = get_turtle_velocity(turtle)
	Time_limit = 5
    # feed current info into the feedback control algorithm
	feasible, output_right, output_left = Optimalcontrol(turtle_z0, turtle_pre, ball_x, ball_y, Time_limit)
	
	print('-------------')
	print(f'Feasible Control: {feasible}')
	print(f'Right Wheel Speed: {output_right[0]}')
	print(f'Left Wheel Speed: {output_left[0]}')

	# print(f"ball position: {p.getBasePositionAndOrientation(elastic_ball)[0]}")
	# apply wheel speeds from the control algorithm output
	p.setJointMotorControl2(turtle,0,p.VELOCITY_CONTROL,targetVelocity=output_left[0],force=1000)
	p.setJointMotorControl2(turtle,1,p.VELOCITY_CONTROL,targetVelocity=output_right[0],force=1000)


