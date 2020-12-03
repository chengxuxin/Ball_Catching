import pybullet as p
import pybullet_data
import time
from ball import *
from TurtleSFOC import *
import struct
import math
from datetime import datetime
from numpy import *
from pylab import *
import sys
import os
import fnmatch
import argparse
import ffmpeg

global_g = -10


def readLogFile(filename, verbose=True):
    f = open(filename, 'rb')

    print('Opened'),
    print(filename)

    keys = f.readline().decode('utf8').rstrip('\n').split(',')
    fmt = f.readline().decode('utf8').rstrip('\n')

    # The byte number of one record
    sz = struct.calcsize(fmt)
    # The type number of one record
    ncols = len(fmt)

    if verbose:
        print('Keys:'),
        print(keys)
        print('Format:'),
        print(fmt)
        print('Size:'),
        print(sz)
        print('Columns:'),
        print(ncols)

    # Read data
    wholeFile = f.read()
    # split by alignment word
    chunks = wholeFile.split(b'\xaa\xbb')
    log = list()
    for chunk in chunks:
        if len(chunk) == sz:
            values = struct.unpack(fmt, chunk)
            record = list()
            for i in range(ncols):
                record.append(values[i])
            log.append(record)

    return log


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

offset = [0, 0, 0]
turtle = p.loadURDF("../model/turtlebot.urdf", offset, globalScaling=1.0)
plane = p.loadURDF("plane.urdf")
# p.setRealTimeSimulation(1)
# create elastic ball
elastic_ball = create_elastic_ball(8, 8, 3)

p.setGravity(0, 0, global_g)
log = readLogFile("LOG0001.txt")
recordNum = len(log)
itemNum = len(log[0])
print('record num:'),
print(recordNum)
print('item num:'),
print(itemNum)

waiting_button = input('Press ENTER to start simulation...')
logId1 = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "playback.mp4")
for record in log:
    Id = record[2]
    pos = [record[3], record[4], record[5]]
    orn = [record[6], record[7], record[8], record[9]]
    p.resetBasePositionAndOrientation(Id, pos, orn)
    turtle_position = p.getBasePositionAndOrientation(turtle)[0]
    # distance, Yaw, Pitch, and target of the camera
    p.resetDebugVisualizerCamera(
        2.5, -30, -40, [turtle_position[0], turtle_position[1], turtle_position[2]])
    numJoints = p.getNumJoints(Id)
    for i in range(numJoints):
        jointInfo = p.getJointInfo(Id, i)
        qIndex = jointInfo[3]
        if qIndex > -1:
            p.resetJointState(Id, i, record[qIndex - 7 + 17])
    time.sleep(0.02)

p.stopStateLogging(logId1)
