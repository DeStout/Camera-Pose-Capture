#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/DisplayRobotState.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_visual_tools/moveit_visual_tools.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_to_poses");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("link_1");
	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();

	Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
	text_pose.translation().z() = 1.75;
	visual_tools.publishText(text_pose, "MoveToPoses", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();

	ROS_INFO_NAMED("MoveToPoses", "Reference frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("MoveToPoses", "End effector link: %s", move_group.getEndEffectorLink().c_str());
	//visual_tools.prompt("Press <next> in the RVizVisualToolsGui window to start");

	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = 1.0;
	target_pose1.position.x = 0.28;
	target_pose1.position.y = -0.2;
	target_pose1.position.z = 0.5;
	move_group.setPoseTarget(target_pose1);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("MoveToPoses", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

	ROS_INFO_NAMED("MoveToPoses", "Visualizing plan 1 as trajectory line");
	visual_tools.publishAxisLabeled(target_pose1, "pose1");
	visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	//visual_tools.prompt("Press <next> in the RVizVisualToolsGui window to continue");

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	joint_group_positions[0] = -1.0;
	move_group.setJointValueTarget(joint_group_positions);

	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("MoveToPoses", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	//visual_tools.prompt("Press <next> in the RVizVisualToolsGui window to continue");

	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "link_6";
	ocm.header.frame_id = "link_1";
	ocm.orientation.w = 1.0;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	move_group.setPathConstraints(test_constraints);

	robot_state::RobotState start_state(*move_group.getCurrentState());
	geometry_msgs::Pose start_pose2;
	start_pose2.orientation.w = 1.0;
	start_pose2.position.x = 0.55;
	start_pose2.position.z = -0.05;
	start_pose2.position.x = 0.8;
	start_state.setFromIK(joint_model_group, start_pose2);

	move_group.setStartState(start_state);
	move_group.setPoseTarget(target_pose1);
	move_group.setPlanningTime(10);

	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("MoveToPoses", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

	visual_tools.deleteAllMarkers();
	visual_tools.publishAxisLabeled(start_pose2, "start");
	visual_tools.publishAxisLabeled(target_pose1, "goal");
	visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	//visual_tools.prompt("next step");

	move_group.clearPathConstraints();
	move_group.setStartStateToCurrentState();

	geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(target_pose3);
	target_pose3.position.z -= 0.2;
	waypoints.push_back(target_pose3);
	target_pose3.position.y -= 0.2;
	waypoints.push_back(target_pose3);
	target_pose3.position.z += 0.2;
	target_pose3.position.y += 0.2;
	target_pose3.position.x += 0.2;
	waypoints.push_back(target_pose3);

	move_group.setMaxVelocityScalingFactor(0.1);
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	ROS_INFO_NAMED("MoveToPoses", "Visualing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

	visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
	for(std::size_t i=0; i<waypoints.size(); i++) {
		visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
	}
	visual_tools.trigger();
	// visual_tools.promt("Press <next> in the RVizVisualToolsGui window to continue");

	ros::shutdown();
	return 0;
}