#!/home/anubhav1772/py38/bin/python
# -*- coding: utf-8 -*-

"""
!/usr/bin/env python
"""

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from copy import deepcopy
import numpy as np
import tf
from tf.transformations import quaternion_from_euler
import time

def all_close(goal, actual, tolerance):
	"""
	Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
	@param: goal       A list of floats, a Pose or a PoseStamped
	@param: actual     A list of floats, a Pose or a PoseStamped
	@param: tolerance  A float
	@returns: bool
	"""
	all_equal = True
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False

	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is geometry_msgs.msg.Pose:
		return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

	return True

class MoveGroupPythonInteface(object):
	def __init__(self):
		super(MoveGroupPythonInteface, self).__init__()

		# initialize `moveit_commander`_ and a `rospy`_ node:
		joint_state_topic = ['joint_states:=/joint_states']
		moveit_commander.roscpp_initialize(joint_state_topic)
		rospy.init_node('move_group_interface', anonymous=True)

		## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
		## kinematic model and the robot's current joint states
		robot = moveit_commander.RobotCommander()

		## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
		## for getting, setting, and updating the robot's internal understanding of the
		## surrounding world:
		scene = moveit_commander.PlanningSceneInterface()

		## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
		## to a planning group (group of joints).  In this tutorial the group is the primary
		## arm joints in the Panda robot, so we set the group's name to "panda_arm".
		## If you are using a different robot, change this value to the name of your robot
		## arm planning group.
		## This interface can be used to plan and execute motions:
		group_name = "arm_manipulator"
		self.move_group = moveit_commander.MoveGroupCommander(group_name)

		## Create a `DisplayTrajectory`_ ROS publisher which is used to display
		## trajectories in Rviz:
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
													moveit_msgs.msg.DisplayTrajectory,
													queue_size=20)

	
		# We can get the name of the reference frame for this robot:
		planning_frame = self.move_group.get_planning_frame()
		print ("============ Planning frame: %s" % planning_frame)

		# We can also print the name of the end-effector link for this group:
		eef_link = self.move_group.get_end_effector_link()
		print ("============ End effector link: %s" % eef_link)

		# We can get a list of all the groups in the robot:
		group_names = robot.get_group_names()
		print ("============ Available Planning Groups:", robot.get_group_names())

		# Sometimes for debugging it is useful to print the entire state of the
		# robot:
		print ("============ Printing robot state")
		print (robot.get_current_state())
		print ("============ End effector pose\n", self.move_group.get_current_pose().pose)
	

		# Misc variables
		self.box_name = ''
		self.robot = robot
		self.scene = scene
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_names

	def go_to_pose_goal(self):
		# Planning to a Pose Goal
		# We can plan a motion for this group to a desired pose for the
		# end-effector:

		target_pose = geometry_msgs.msg.PoseStamped()
		target_pose.header.frame_id = "/base_link"
		target_pose.pose.position.x = -0.24334
		target_pose.pose.position.y = 0.42153
		target_pose.pose.position.z = 0.67139
		# target_pose.pose.orientation.x = -0.005879
		# target_pose.pose.orientation.y = 0.002818
		# target_pose.pose.orientation.z = 0.865909
		# target_pose.pose.orientation.w = 0.500158
		(target_pose.pose.orientation.x,  target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w) = quaternion_from_euler(0.0, 0.0, 2.0944)
		self.move_group.set_pose_target(target_pose)
		# self.move_group.setApproximateJointValueTarget(target_pose, "EEF_Link")
		
		# Now, we call the planner to compute the plan and execute it.
		self.move_group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		# self.move_group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		# self.move_group.clear_pose_targets()
		#rospy.spin()
		# time.sleep(1.0)
		
		# target_pose.pose.position.x = -0.28386
		# target_pose.pose.position.y = 0.51214
		# target_pose.pose.position.z = 0.2119
		# (target_pose.pose.orientation.x,  target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w) = quaternion_from_euler(0.0, 0.86289, 2.0944)

		# self.move_group.set_pose_target(target_pose)
		# self.move_group.go(wait=True)
		self.move_group.stop()


	
	def go_to_home_pose(self):
		# Planning to home position
		# joint_goal = self.move_group.get_current_joint_values()
		# joint_goal[0] = 1.95118
		# joint_goal[1] = 1.11642
		# joint_goal[2] = -1.41584
		# joint_goal[3] = -0.21243
		# joint_goal[4] = 3.13971
		# joint_goal[4] = 3.13971

		# self.move_group.go(joint_goal, wait=True)
		# self.move_group.stop()

		home_pose = geometry_msgs.msg.PoseStamped()
		home_pose.header.frame_id = "base_link"

		home_pose.pose.position.x = 0.0
		home_pose.pose.position.y = -0.31845
		home_pose.pose.position.z = 1.35036
		home_pose.pose.orientation.x = 0.69
		home_pose.pose.orientation.y = 0.299
		home_pose.pose.orientation.z = 0.39
		home_pose.pose.orientation.w = 0.5298

		self.move_group.set_pose_target(home_pose)
		self.move_group.set_goal_tolerance(0.1)
		# self.move_group.set_goal_position_tolerance(0.1)
		self.move_group.set_planner_id("RRTConnect")
		self.move_group.go(wait=True)
		self.move_group.stop()
		self.move_group.clear_pose_targets()

	def go_to_joint_state(self, joint_values):
		# We can get the joint values from the group and adjust some of the values
		# Here, joint_values is a map containing robot joint values
		joint_goal = self.move_group.get_current_joint_values()
		joint_goal = list(joint_values)

		self.move_group.go(joint_goal, wait=True)
		self.move_group.stop()

		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		self.move_group.clear_pose_targets()

		# For testing:
		# Note that since this section of code will not be included in the tutorials
		# we use the class variable rather than the copied state variable
		current_pose = self.move_group.get_current_joint_values()
		return all_close(joint_goal, current_pose, 0.1)

	def run_cartesian_trajectory(self):
		# Set the reference frame for pose targets
		reference_frame = "/base_link"

		# Set the reference frame accordingly
		self.move_group.set_pose_reference_frame(reference_frame)

		# Allow replanning to increase the odds of a solution
		self.move_group.allow_replanning(True)

		# Allow some leeway in position (meters) and orientation (radians)
		self.move_group.set_goal_position_tolerance(0.01)
		self.move_group.set_goal_orientation_tolerance(0.1)

		# Get the current pose so we can add it as a waypoint
		start_pose = self.move_group.get_current_pose(self.eef_link).pose

		# Initialize the waypoints list
		waypoints = []

		# Set the first waypoint to be the starting pose
		# Append the pose to the waypoints list
		wpose = deepcopy(start_pose)

		# Set the next waypoint to the right 0.5 meters
		wpose.position.x = -0.1
		wpose.position.y = -0.2
		wpose.position.z = 0.15

		waypoints.append(deepcopy(wpose))
		if np.sqrt((wpose.position.x-start_pose.position.x)**2+(wpose.position.x-start_pose.position.x)**2 \
			+(wpose.position.x-start_pose.position.x)**2)<0.1:
			rospy.loginfo("Warnig: target position overlaps with the initial position!")

		self.move_group.set_start_state_to_current_state()

		plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0, True)

		# If we have a complete plan, execute the trajectory
		if 1-fraction < 0.2:
			rospy.loginfo("Path computed successfully. Moving the arm.")
			num_pts = len(plan.joint_trajectory.points)
			rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
			self.move_group.execute(plan, wait=True)
		
			self.move_group.stop()
			
			# It is always good to clear your targets after planning with poses.
			# Note: there is no equivalent function for clear_joint_value_targets()
			self.move_group.clear_pose_targets()
			rospy.loginfo("Path execution complete.")
		else:
			rospy.loginfo("Path planning failed")

	def add_box(self, box_name, pos, dim, timeout=4):
		# dim is tuple of length, width and height, i.e.,(length, width and height)
		# pose is tuple of x, y and z i.e.,(x, y, z)
		rospy.sleep(2)
		# Adding Objects to the Planning Scene
		box_pose = geometry_msgs.msg.PoseStamped()
		box_pose.header.frame_id = self.robot.get_planning_frame()
		box_pose.pose.orientation.w = 1.0
		box_pose.pose.position.x, box_pose.pose.position.y, box_pose.pose.position.z = pos
		# self.box_name = box_name
		self.scene.add_box(box_name, box_pose, size=dim)
		return self.wait_for_state_update(box_name, "add", box_is_known=True, timeout=timeout)
	
	def wait_for_state_update(self, box_name, operation, box_is_known=False, box_is_attached=False, timeout=4):
		if operation=="add":
			print("Waiting for state update after adding {} to the scene ...".format(box_name))
		else:
			print("Waiting for state update after removing {} from the scene ...".format(box_name))
		
		start = rospy.get_time()
		seconds = rospy.get_time()
		while (seconds - start < timeout) and not rospy.is_shutdown():
			# Test if the box is in attached objects
			attached_objects = self.scene.get_attached_objects([self.box_name])
			is_attached = len(attached_objects.keys()) > 0

			# Test if the box is in the scene.
			# Note that attaching the box will remove it from known_objects
			is_known = self.box_name in self.scene.get_known_object_names()

			# Test if we are in the expected state
			if (box_is_attached == is_attached) and (box_is_known == is_known):
				return True

			# Sleep so that we give other threads time on the processor
			rospy.sleep(0.1)
			seconds = rospy.get_time()

		# If we exited the while loop without returning then we timed out
		return False
	
	def remove_box(self, box_name, timeout=4):
		# Removing Objects from the Planning Scene
		# We can remove the box from the world.
		self.scene.remove_world_object(box_name)

		# **Note:** The object must be detached before we can remove it from the world
		# We wait for the planning scene to update.
		return self.wait_for_state_update(box_name, "remove", box_is_attached=False, box_is_known=False, timeout=timeout)
			

def main():
	try:
		interface = MoveGroupPythonInteface()
		# interface.add_box("table_front", (0.85, 0, 0), (0.5, 1.0, 0.2))
		# interface.add_box("table_right", (-0.004783, -0.739666, 0), (1.0, 0.5, 0.2))
		# interface.add_box("table_left", (-0.004783, 0.739666, 0), (1.0, 0.5, 0.2))
		# interface.add_box("object", (0.0, -1.0, 0.3), (0.1, 0.075, 0.05))
		# interface.go_to_home_pose()
		# interface.go_to_pose_goal()     
		# interface.remove_box("table_front")
		# interface.remove_box("table_right")
		# interface.remove_box("table_left")
		# interface.go_to_joint_state([0.0, pi/2, 0.0, 0.0, 0.0, -pi/6])
		# interface.go_to_joint_state([0.0, 0.0, 0.0, 0.0, pi/6, 0.0])
		# interface.go_to_joint_state([0.0, 0.0, 0.0, 0.0, -pi/6, 0.0])

		# interface.run_cartesian_trajectory()
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__=='__main__':
	main()

