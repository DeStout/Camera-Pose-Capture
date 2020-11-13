#!/usr/bin/env python

import rospy
import rospkg
import rosbag

# Opens the .bag of captured JointStates and prints them out
if __name__ == '__main__':
	rospack = rospkg.RosPack()
	pkgpath = rospack.get_path('camera_pose_capture')
	bag = rosbag.Bag(pkgpath + '/calibration_poses.bag')

	joint_states = []
	for topic, msg, t in bag.read_messages(topics = []):
		joint_states.append(msg.joint_states.position)
	print(joint_states)
	bag.close()
