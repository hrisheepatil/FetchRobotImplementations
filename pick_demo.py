#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations
import math
import moveit_commander
import fetch_api
import joint_state_reader
import copy
import moveit_msgs.msg
import moveit_msgs.srv
from urdf_parser_py.urdf import URDF
from gazebo_msgs.srv import GetLinkState, GetModelState
from sensor_msgs.msg import JointState
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import graspit_commander
import geometry_msgs
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class pick_demo(object):

	def show_states(self):
		# Get states of links
		link_coordinates = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		
		# Get joint States
		#rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
		self.robot = URDF.from_parameter_server()
		self.base = self.robot.get_root()
		

		# Wait for moveit IK service
		rospy.wait_for_service("compute_ik")
		self.ik_service = rospy.ServiceProxy('compute_ik', moveit_msgs.srv.GetPositionIK)
		
		
		# MoveIt parameter
		robot_moveit = moveit_commander.RobotCommander()
		self.group_name = robot_moveit.get_group_names()[0]
		self.all_groups = robot_moveit.get_group_names()
		print(self.all_groups)
		self.group_joint_name = robot_moveit.get_joint_names(group=self.group_name)

		return model_coordinates('demo_cube', 'world'), model_coordinates('fetch', 'world'), link_coordinates('wrist_roll_link', 'base_link'), self.group_name, self.group_joint_name, self.base

	def show_states_table(self):
		model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		return model_coordinates('table1', 'world')

	def calculate_grasp(self):
		grasp_comm = graspit_commander.GraspitCommander()
		grasp_comm.clearWorld()
		grasp_comm.toggleAllCollisions(True)
		 
		grasp_comm.importObstacle("floor")

		grasp_comm.importRobot('fetch_gripper')

		pro = geometry_msgs.msg.Pose()
		(pro.orientation.x, pro.orientation.y, pro.orientation.z, pro.orientation.w) = tf.transformations.quaternion_from_euler(0,math.pi/2, 0)
		[pro.position.x, pro.position.y, pro.position.z] = [2.5, 4, 0.090] 
		grasp_comm.importGraspableBody("longBox", pose = pro)

		grasps = grasp_comm.planGrasps()

		robot = grasp_comm.getRobots().ids[0]

		final_pose = grasp_comm.getRobot(robot).robot.pose
		return final_pose
	
	def calculate_transform(self, cube, base):
		cube_trans = np.dot(tf.transformations.translation_matrix((cube.pose.position.x, cube.pose.position.y, cube.pose.position.z)), tf.transformations.quaternion_matrix([cube.pose.orientation.x, cube.pose.orientation.y, cube.pose.orientation.z, cube.pose.orientation.w]))
		base_trans = np.dot(tf.transformations.translation_matrix((base.pose.position.x, base.pose.position.y, base.pose.position.z)), tf.transformations.quaternion_matrix([base.pose.orientation.x, base.pose.orientation.y, base.pose.orientation.z, cube.pose.orientation.w]))
		result_trans = np.dot(tf.transformations.inverse_matrix(base_trans), cube_trans)
		return result_trans

	def calculate_transform_early(self, cube, base):
		cube_trans = np.dot(tf.transformations.translation_matrix((cube.pose.position.x - 1.1, cube.pose.position.y + 0.25, cube.pose.position.z + 0.1)), tf.transformations.quaternion_matrix([cube.pose.orientation.x, cube.pose.orientation.y, cube.pose.orientation.z, cube.pose.orientation.w]))
		base_trans = np.dot(tf.transformations.translation_matrix((base.pose.position.x, base.pose.position.y, base.pose.position.z)), tf.transformations.quaternion_matrix([base.pose.orientation.x, base.pose.orientation.y, base.pose.orientation.z, cube.pose.orientation.w]))
		result_trans = np.dot(tf.transformations.inverse_matrix(base_trans), cube_trans)
		return result_trans

	def convert_to_message(self,T):
		t = geometry_msgs.msg.Pose()
		position = tf.transformations.translation_from_matrix(T)
		orientation = tf.transformations.quaternion_from_matrix(T)
		t.position.x = position[0]
		t.position.y = position[1]
		t.position.z = position[2]
		t.orientation.x = orientation[0]
		t.orientation.y = orientation[1]
		t.orientation.z = orientation[2]
		t.orientation.w = orientation[3]        
		return t

	def grasp_to_body(self, bTc, msg): 
		x_trans = msg.position.x - 2.5 - 0.022
		y_trans = msg.position.y - 4
		z_trans = msg.position.z - 0.090		 
		x_rot = msg.orientation.x
		y_rot = msg.orientation.y
		z_rot = msg.orientation.z
		w_rot = msg.orientation.w
		cTg = np.dot(tf.transformations.translation_matrix((x_trans, y_trans, z_trans)), tf.transformations.quaternion_matrix([x_rot, y_rot, z_rot, w_rot]))
		result_trans = np.dot(bTc, cTg)
		return result_trans

	def IK(self, T_goal):
		req = moveit_msgs.srv.GetPositionIKRequest()
		req.ik_request.group_name = self.group_name
		req.ik_request.robot_state = moveit_msgs.msg.RobotState()
		req.ik_request.robot_state.joint_state.name = self.group_joint_name
		req.ik_request.robot_state.joint_state.position = np.zeros(7)
		req.ik_request.robot_state.joint_state.velocity = np.zeros(7)
		req.ik_request.robot_state.joint_state.effort = np.zeros(7)
		req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
		req.ik_request.avoid_collisions = True
		req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
		req.ik_request.pose_stamped.header.frame_id = 'base_link'
		req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
		req.ik_request.pose_stamped.pose = T_goal
		req.ik_request.timeout = rospy.Duration(3.0)
		res = self.ik_service(req)
		joint_state = res.solution.joint_state
		joints = []
		for name, position in zip(joint_state.name, joint_state.position):
            		if name in fetch_api.ArmJoints.names():
                		joints.append((position))
        	return joints				

if __name__ == "__main__":
	rospy.init_node("pick_demo")
	dist = 0
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	planning_scene = PlanningSceneInterface('base_link')
	planning_scene.clear()
	planning_scene.removeCollisionObject('cube')
	planning_scene.removeCollisionObject('table')
	group = moveit_commander.MoveGroupCommander("arm")
	planning_frame = group.get_planning_frame()
	pd = pick_demo()
	grasp = pd.calculate_grasp()	
	base = fetch_api.Base()
	arm = fetch_api.Arm()
	torso = fetch_api.Torso()
	gripper = fetch_api.Gripper()	
	torso.set_height(0.4)
	joint_goal = group.get_current_joint_values()
	joint_goal = [0,0,0,-math.pi/2,0,math.pi/2,0]
	group.go(joint_goal, wait=True)
	group.stop()
	get_ik = []
	while get_ik == []:
		base.go_forward(0.1, 0.1)
		dist = dist + 0.1
		target, origin, wrist, group_name, group_joint_name, base_link = pd.show_states()
		final_trans = pd.calculate_transform(target, origin)
		final_grasp = pd.grasp_to_body(final_trans, grasp)
		final_msg_trans = pd.convert_to_message(final_grasp)
		get_ik = pd.IK(final_msg_trans)
	tablemsg = pd.show_states_table()
	tablest = pd.calculate_transform(tablemsg, origin)
	table = pd.convert_to_message(tablest)
	planning_scene.removeCollisionObject('table')
	table_size_x = 0.913
	table_size_y = 0.913
	table_size_z = 0.040
	table_x = table.position.x
	table_y = table.position.y
	table_z = table.position.z + 0.74
	planning_scene.addBox('table', table_size_x, table_size_y, table_size_z,
			  table_x, table_y, table_z)
	target = pd.convert_to_message(final_trans)
	planning_scene.removeCollisionObject('cube')
	cube_size_x = 0.044
	cube_size_y = 0.044
	cube_size_z = 0.180
	cube_x = target.position.x
	cube_y = target.position.y
	cube_z = target.position.z
	planning_scene.addBox('cube', cube_size_x, cube_size_y, cube_size_z,
			  cube_x, cube_y, cube_z)
	final_grasp = pd.grasp_to_body(final_trans,grasp)
	fmt = pd.convert_to_message(final_grasp)
	pose_goal = geometry_msgs.msg.Pose()	
	pose_goal.position.x = fmt.position.x
	pose_goal.position.y = fmt.position.y
	pose_goal.position.z = fmt.position.z
	pose_goal.orientation.x = fmt.orientation.x
	pose_goal.orientation.y = fmt.orientation.y
	pose_goal.orientation.z = fmt.orientation.z
	pose_goal.orientation.w = fmt.orientation.w
	group.set_pose_target(pose_goal)
	plan = group.go(wait=True)		
	group.stop()
	group.clear_pose_targets()
	gripper.close()
	joint_goal = group.get_current_joint_values()
	joint_goal = [0,0,0,-math.pi/2,0,math.pi/2,0]
	group.go(joint_goal, wait=True)
	group.stop()
	base.go_forward(-dist,0.5)
