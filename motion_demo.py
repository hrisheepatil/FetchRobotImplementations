#! /usr/bin/env python

import math
import fetch_api
import rospy

def main():
    rospy.init_node('motion_demo')
    base = fetch_api.Base()
    fwd = 1.5
    base.go_forward(fwd,1)
    head = fetch_api.Head()
    head.pan_tilt(0, math.pi/4)
    head = fetch_api.Head()
    head.pan_tilt(0, -math.pi /2)
    head.pan_tilt(math.pi /2, 0)
    head.pan_tilt(-math.pi / 2, 0)
    head.pan_tilt(0, 0)
    torso = fetch_api.Torso()
    torso.set_height(torso.MAX_HEIGHT)
    arm = fetch_api.Arm()
    arm.move_to_joints(fetch_api.ArmJoints.from_list([0,0,0,0,0,0,0]))
    gripper = fetch_api.Gripper()
    gripper.close()
    gripper.open()

if __name__ == '__main__':
    main()
