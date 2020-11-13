#!/usr/bin/env python

import string

import rospy
import rospkg
import rosbag

from sensor_msgs.msg import JointState
from robot_calibration_msgs.msg import CaptureConfig

# Assumes that abb_irb1200_5_90_moveit_config demo.launch is running
# Execute with "rosrun camera_pose_capture capture_poses.py"
# Subscribes to JointState and records msgs into a .bag file
# .bag is saved to the root of the package
# Rough UI loop allows user to dictate what poses are captured
class CapturePoses:
    last_state_ = None # last joint states

    def __init__(self):
        self.last_state_ = CaptureConfig()

        rospy.init_node('capture_calibration_poses')
        rospy.Subscriber("joint_states", JointState, self.stateCb)

        # bag to write data to
        rospack = rospkg.RosPack()
        pkgpath = rospack.get_path('camera_pose_capture')
        bag = rosbag.Bag(pkgpath + '/calibration_poses.bag', 'w')

        # put samples in
        count = 0
        while not rospy.is_shutdown():
            print('Move arm/head to a new sample position:  (type "exit" to quit and save data)')
            resp = raw_input('press <enter> ')
            if string.upper(resp) == 'EXIT':
                break
            else:
                if self.last_state_ == None:
                    print('Cannot save state')
                    continue
                # save a pose
                print(self.last_state_)
                bag.write('/calibration_joint_states', self.last_state_)
                print("Saving pose %d\n" % count)
                count += 1
        bag.close()

    def stateCb(self, msg):
        """ Callback for joint_states messages """
        for joint, position in zip(msg.name, msg.position):
            try:
                idx = self.last_state_.joint_states.name.index(joint)
                self.last_state_.joint_states.position[idx] = position
            except ValueError:
                self.last_state_.joint_states.name.append(joint)
                self.last_state_.joint_states.position.append(position)

if __name__ == '__main__':
    CapturePoses()
