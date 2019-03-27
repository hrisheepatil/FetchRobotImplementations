#!/usr/bin/env python

import graspit_commander
import geometry_msgs
from tf.transformations import quaternion_from_euler
import math
import rospy
import time


grasp_comm = graspit_commander.GraspitCommander()
grasp_comm.clearWorld()
grasp_comm.toggleAllCollisions(True)
 
grasp_comm.importObstacle("floor")

grasp_comm.importRobot('fetch_gripper')

pro = geometry_msgs.msg.Pose()
(pro.orientation.x, pro.orientation.y, pro.orientation.z, pro.orientation.w) = quaternion_from_euler(0,math.pi/2, 0)
[pro.position.x, pro.position.y, pro.position.z] = [2.5, 4, 0.090] 
grasp_comm.importGraspableBody("longBox", pose = pro)

grasps = grasp_comm.planGrasps()

robot = grasp_comm.getRobots().ids[0]

final_pose = grasp_comm.getRobot(robot).robot.pose

















