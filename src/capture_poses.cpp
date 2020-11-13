#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/JointState.h"
#include "rosbag/bag.h"

#include <iostream>
#include <string>
using namespace std;

/*
 * Helper class used to store robot joint positions and write them to a .bag file
 */
class JointCapture {
public:
	static sensor_msgs::JointState joint_state;
	static void JointCallback(const sensor_msgs::JointState &msg);
	static void WriteToBag(rosbag::Bag *bag);
};

inline sensor_msgs::JointState CreateJointState() {
	sensor_msgs::JointState temp_msg;
	temp_msg.name.push_back("");
	temp_msg.position.push_back(0);
	return temp_msg;
};
sensor_msgs::JointState JointCapture::joint_state = CreateJointState();

void JointCapture::JointCallback(const sensor_msgs::JointState &msg) {
	JointCapture::joint_state = msg;
}

void JointCapture::WriteToBag(rosbag::Bag *bag) {
	bag->write("/joint_states", ros::Time::now(), JointCapture::joint_state);
}

/*
 * Debug function
 * Outputs the name and position of every joint being captured
 */
void PrintJointState() {
	cout << "\n";
	for(int i=0; i<JointCapture::joint_state.name.size(); i++) {
		ROS_INFO("%s: %f", JointCapture::joint_state.name[i].c_str(), JointCapture::joint_state.position[i]);
	}
}

void PrintHelp() {
	ROS_INFO("Press <ENTER> to capture a pose.");
	ROS_INFO("Type \"Exit\" to quit.");
	ROS_INFO("Type \"Help\" to repeat these messages.");
}

/*
 * Assumes that abb_irb1200_5_90_moveit_config demo.launch is running
 * Execute with "rosrun camera_pose_capture capture_poses"
 * Subscribes to JointState and records msgs into a .bag file
 * .bag is saved to the root of the package
 * Rough UI loop allows user to dictate what poses are captured
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "capture_poses");
	ros::NodeHandle nh;
	ros::Subscriber joint_sub = nh.subscribe("joint_states", 1, JointCapture::JointCallback);

	rosbag::Bag bag;
	string bag_path = ros::package::getPath("camera_pose_capture");
	bag.open(bag_path + "/calibration_poses_cpp.bag", rosbag::bagmode::Write);

	string input;
	PrintHelp();
	//PrintJointState();
	while(ros::ok())
	{
		cout << "Input: ";
		getline(cin, input);
		for_each(input.begin(), input.end(), [](char & c){
    		c = ::tolower(c);
		});

		if(input.empty())
		{
			ROS_INFO("-- Capturing Pose --");
			ros::spinOnce();
			JointCapture::WriteToBag(&bag);
			//PrintJointState();

		}
		else if(input == "exit")
		{
			ROS_INFO("Saving .bag and quitting");
			break;
		}
		else if(input == "help")
		{
			PrintHelp();
		}
		else
		{
			ROS_INFO("Oh ffs what?");
		}
	}

	bag.close();
	return 0;
}