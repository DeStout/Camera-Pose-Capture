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

def all_close(goal, actual, tolerance):
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

	def go_to_joint_state(self):
		joint_goal = self.move_group.get_current_joint_values()
		# joint_goal[0] = pi / 6
		# joint_goal[1] = -pi / 4
		# joint_goal[2] = 0
		# joint_goal[3] = -pi / 2
		# joint_goal[4] = 0
		# joint_goal[5] = pi / 3
		
		joint_goal[0] = 0
		joint_goal[1] = 0
		joint_goal[2] = 0
		joint_goal[3] = 0
		joint_goal[4] = 0
		joint_goal[5] = 0

		self.move_group.go(joint_goal, wait=True)
		self.move_group.stop()

		# Testing:
		# current_joints = self.move_group.get_current_joint_values()
		# return all_close(joint_goal, current_joints, 0.01)

	def move_to_bag_poses(self):
		rospack = rospkg.RosPack()
		pkgpath = rospack.get_path('camera_pose_capture')
		bag = rosbag.Bag(pkgpath + '/calibration_poses.bag')

		joint_states = []
		for topic, msg, t in bag.read_messages(topics = []):
			joint_states.append(msg.joint_states.position)
		# print(joint_states)
		bag.close()
		
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

	def go_to_pose_goal(self):
		# pose_goal = geometry_msgs.msg.Pose()
		# pose_goal.orientation.w = 1.0
		# pose_goal.position.x = 0.4
		# pose_goal.position.y = 0.1
		# pose_goal.position.z = 0.4
		random_pose = self.move_group.get_random_pose()
		
		self.move_group.set_pose_target(random_pose.pose)
		plan = self.move_group.go(wait=True)
		
		self.move_group.stop()
		self.move_group.clear_pose_targets()

		# Testing:
		# current_pose = self.move_group.get_current_pose().pose
		# return all_close(pose_goal, current_pose, 0.01)

	def plan_cartesian_path(self, scale = 1):
		waypoints = []
		
		wpose = self.move_group.get_current_pose().pose
		wpose.position.z -= scale * 0.1
		wpose.position.y += scale * 0.2
		waypoints.append(copy.deepcopy(wpose))

		wpose.position.x += scale * 0.1
		waypoints.append(copy.deepcopy(wpose))
		
		wpose.position.y -= scale * 0.1
		waypoints.append(copy.deepcopy(wpose))
		
		(plan, fraction) = self.move_group.compute_cartesian_path(
												waypoints,
												0.01,
												0.0)
		return plan, fraction

	def display_trajectory(self, plan):
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory.trajectory_start = self.robot.get_current_state()
		display_trajectory.trajectory.append(plan)
		self.display_trajectory_publisher.publish(display_trajectory)

	def execute_plan(self, plan):
		self.move_group.execute(plan, wait = True)
		

def main():
	try:
		print("-================== Press <Enter> to begin ==================-")
		raw_input()
		tutorial = CaptureCamera()

		print("-======= Press <Enter> to move to 0 joint state goal ========-")
		raw_input()
		tutorial.go_to_joint_state()

		# print("-============ Press <Enter> to move to pose goal ============-")
		# raw_input()
		# tutorial.go_to_pose_goal()

		# print("-===== Press <Enter> to plan and display cartesian path =====-")
		# raw_input()
		# cartesian_plan, fraction = tutorial.plan_cartesian_path()

		# print("-=========== Press <Enter> to replay cartesian path =========-")
		# raw_input()
		# tutorial.display_trajectory(cartesian_plan)

		# print("-=========== Press <Enter> to execute a saved path ==========-")
		# raw_input()
		# tutorial.execute_plan(cartesian_plan)

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
