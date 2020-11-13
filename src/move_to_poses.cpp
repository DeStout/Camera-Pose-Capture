#include "ros/ros.h"
#include "ros/package.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/JointState.h"
#include "moveit/move_group_interface/move_group_interface.h"

#include <string>
#include <vector>

using namespace std;

/*
 * Debugging function
 * Outputs the name and position of every joint in the message provided
 * Assumes message parameter is a JointState
 */
void PrintMsg(sensor_msgs::JointState::ConstPtr joint_state) {
	ROS_INFO("Seq: %i", joint_state->header.seq);
	for(int i=0; i<joint_state->name.size(); i++) {
		ROS_INFO("%s: %f", joint_state->name[i].c_str(), joint_state->position[i]);
	}
	cout << "\n";	
}

/*
 * Opens recorded joint poses .bag from package root
 * Iterates through file and stores saved poses into a vector
 * Returns the vector
 */
vector<sensor_msgs::JointState::ConstPtr> ReadBag() {
	rosbag::Bag bag;
	string bag_path = ros::package::getPath("camera_pose_capture");
	bag.open(bag_path + "/calibration_poses_cpp.bag");

	vector<sensor_msgs::JointState::ConstPtr> joint_states;

	for(rosbag::MessageInstance const msg : rosbag::View(bag))
	{
		sensor_msgs::JointState::ConstPtr joint_state = msg.instantiate<sensor_msgs::JointState>();
		//PrintMsg(joint_state);
		joint_states.push_back(joint_state);
	}
	return joint_states;
}

/*
 * Assumes that abb_irb1200_5_90_moveit_config demo.launch is running
 * Execute with "rosrun camera_pose_capture move_to_poses"
 * Connects to manipulator group and moves through saved poses
 */
int main(int argc, char **argv)
{
	vector<sensor_msgs::JointState::ConstPtr> joint_states = ReadBag();

	ros::init(argc, argv, "move_to_poses");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	for(int i=0; i<joint_states.size(); i++) {
		for(int j=0; j<joint_group_positions.size(); j++)
			joint_group_positions[j] = joint_states[i]->position[j];
		
		move_group.setJointValueTarget(joint_group_positions);
		bool success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO("Move to joint value target %s", success ? "SUCCEEDED" : "FAILED");	
	}
	return 0;
}
