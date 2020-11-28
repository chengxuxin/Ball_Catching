import pybullet as p
import pybullet_data
import time
from ball import *

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
offset = [0,0,0]
turtle = p.loadURDF("../model/turtlebot.urdf",offset)
plane = p.loadURDF("plane.urdf")
p.setRealTimeSimulation(1)
# create elastic ball
elastic_ball = create_elastic_ball(1.5, 1.5, 5)
# elastic collision with the floor plane
p.changeDynamics(plane, -1, restitution=1)


for j in range (p.getNumJoints(turtle)):
	print(p.getJointInfo(turtle,j))
forward=0
turn=0
while (1):
	p.setGravity(0,0,-10)
	time.sleep(1./240.)
	keys = p.getKeyboardEvents()
	leftWheelVelocity=0
	rightWheelVelocity=0
	speed=10
	# print(f"ball position: {p.getBasePositionAndOrientation(elastic_ball)[0]}")

	for k,v in keys.items():

                if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                        turn = -0.5
                if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
                        turn = 0
                if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                        turn = 0.5
                if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
                        turn = 0

                if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                        forward=1
                if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
                        forward=0
                if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                        forward=-1
                if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
                        forward=0

	rightWheelVelocity+= (forward+turn)*speed
	leftWheelVelocity += (forward-turn)*speed
	
	p.setJointMotorControl2(turtle,0,p.VELOCITY_CONTROL,targetVelocity=leftWheelVelocity,force=1000)
	p.setJointMotorControl2(turtle,1,p.VELOCITY_CONTROL,targetVelocity=rightWheelVelocity,force=1000)


