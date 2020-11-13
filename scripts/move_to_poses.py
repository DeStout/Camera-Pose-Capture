#!/usr/bin/env python
import sys
import copy
import rospy
import rospkg
import rosbag
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from moveit_commander.conversions import pose_to_list

# Based off the ROS Python moveit tutorial
# Assumes that abb_irb1200_5_90_moveit_config demo.launch is running
# Executes with "rosrun camera_pose_capture move_to_poses.py"
class CaptureCamera(object):
	"""CaptureCamera"""
	def __init__(self):
		super(CaptureCamera, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('camera_capture', anonymous = True)

		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()

		group_name = "manipulator"
		move_group = moveit_commander.MoveGroupCommander(group_name)

		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 20)

		planning_frame = move_group.get_planning_frame()
		print("-==================== Reference Frame: %s ====================-" % planning_frame)
		eef_link = move_group.get_end_effector_link()
		print("-======================== End Effector: %s =======================-" % eef_link)
		group_names = robot.get_group_names()
		print("-================== Robot Groups: %s ===================-" % robot.get_group_names())

		print("-======================= Printing Robot State =======================-")
		print(robot.get_current_state())
		
		self.box_name = ''
		self.robot = robot
		self.scene = scene
		self.move_group = move_group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_names

	# Iterates through .bag file and moves robot to the JointStates
	def move_to_bag_poses(self):
		rospack = rospkg.RosPack()
		pkgpath = rospack.get_path('camera_pose_capture')
		bag = rosbag.Bag(pkgpath + '/calibration_poses.bag')

		joint_states = []
		for topic, msg, t in bag.read_messages(topics = []):
			joint_states.append(msg.joint_states.position)
		# print(joint_states)
		bag.close()
		print("Moving to pose [1 of {}]".format(len(joint_states)))
		
		i = 0
		while i < len(joint_states):
			joint_goal = joint_states[i]
			self.move_group.go(joint_goal, wait = True)
			i += 1
			if i < len(joint_states):
				print("Press <Enter> to move to the next pose [{} of {}]".format(i+1, len(joint_states)))
				raw_input()
				print(joint_states[i])
		self.move_group.stop()

def main():
	try:
		print("-================== Press <Enter> to begin ==================-")
		raw_input()
		tutorial = CaptureCamera()

		print("-============ Press <Enter> to move to .bag poses ===========-")
		raw_input()
		tutorial.move_to_bag_poses()
		
		print("-==================== Tutorial Completed ====================-")
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()
