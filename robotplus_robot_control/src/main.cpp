#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>

#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>

moveit::planning_interface::MoveGroupInterface *move_group;
bool run = false;
Eigen::Matrix3d err_mat = Eigen::Matrix3d::Identity();
Eigen::Vector3d err_vec = Eigen::Vector3d::Zero();

void movej(double cmd[6]){
	std::vector<double> target_joint(6, 0);
	memcpy(target_joint.data(), cmd, sizeof(double) * 6);
	move_group->setJointValueTarget(target_joint);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (success)
	{
		ROS_INFO("Motion plan succeeded. Executing...");
		move_group->execute(my_plan);
	}
	else
	{
		ROS_ERROR("Motion planning failed.");
	}
}

void movel_machine(double cmd[7]){
	Eigen::Matrix3d mat;
	Eigen::Vector3d vec(cmd[0], cmd[1], cmd[2]);
	// Eigen::Vector4d quat(cmd[3], cmd[4], cmd[5], cmd[6]);
	// mat = quat2mat(quat);
	Eigen::Quaterniond quat(cmd[6], cmd[3], cmd[4], cmd[5]);
	mat = quat.toRotationMatrix();

	std::cout << "ori command : " << vec.transpose() << " " << quat.w() << " " << quat.vec().transpose() << std::endl;

	Eigen::Matrix3d target_mat;
	Eigen::Vector3d target_vec;
	target_mat = err_mat*mat;
	target_vec = err_mat*(vec - err_vec);

	// Eigen::Vector4d target_quat;
	// target_quat = mat2quat(target_mat);
	Eigen::Quaterniond target_quat(target_mat);

	std::cout << "cor command : " << target_vec.transpose() << " "  << target_quat.w() << " " << target_quat.vec().transpose() << std::endl;

	geometry_msgs::Pose target_pose;
	target_pose.position.x = target_vec(0);
	target_pose.position.y = target_vec(1);
	target_pose.position.z = target_vec(2);
	target_pose.orientation.x = target_quat.x();
	target_pose.orientation.y = target_quat.y();
	target_pose.orientation.z = target_quat.z();
	target_pose.orientation.w = target_quat.w();

	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(target_pose);

	moveit_msgs::RobotTrajectory trajectory;
	double jump_threshold = 0.1;
	double eef_step = 0.01;
	double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
	ROS_INFO("Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

	move_group->execute(trajectory);
}

void movel_base(double cmd[7]){
	geometry_msgs::Pose target_pose;
	target_pose.position.x = cmd[0];
	target_pose.position.y = cmd[1];
	target_pose.position.z = cmd[2];
	target_pose.orientation.x = cmd[3];
	target_pose.orientation.y = cmd[4];
	target_pose.orientation.z = cmd[5];
	target_pose.orientation.w = cmd[6];

	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(target_pose);

	moveit_msgs::RobotTrajectory trajectory;
	double jump_threshold = 0.1;
	double eef_step = 0.01;
	double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
	ROS_INFO("Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

	move_group->execute(trajectory);
}

static void *logging(void *arg)
{
	std::ofstream fout;
	fout.open("/home/keti/logging.txt");

	std::ofstream fout_joint;
	fout_joint.open("/home/keti/logging_jnt.txt");
	double t_stamp = 0;
	while (run)
	{
		geometry_msgs::Pose current_pose = move_group->getCurrentPose().pose;

		// ROS_INFO("current pose : %f, %f, %f, %f, %f, %f, %f", current_pose.position.x, current_pose.position.y, current_pose.position.z,
		// 		 current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
		
		fout << ros::Time::now() << ", ";
		fout << current_pose.position.x << ", ";
		fout << current_pose.position.y << ", ";
		fout << current_pose.position.z << ", ";
		fout << current_pose.orientation.x << ", ";
		fout << current_pose.orientation.y << ", ";
		fout << current_pose.orientation.z << ", ";
		fout << current_pose.orientation.w << ", ";
		fout << "\n";

		std::vector<double> current_joint = move_group->getCurrentJointValues();
		fout_joint << t_stamp << ", ";
		for(int i = 0; i < 6; i++){
			fout_joint << current_joint[i] << ", ";
		}
		fout_joint << "\n";

		ros::Duration(0.01).sleep();
		t_stamp += 0.01;
	}

	fout.close();

	ROS_INFO("logging thread finished");

	return nullptr;
}

int main(int argc, char **argv)
{
	// init
	// double init_joint[6] = {-1.57, 0.8478, -1.8212, 0.7222, 1.6642, 0};
	double init_joint[6] = {0.234, -0.937, 2.272, -1.279, -1.832, -0.015};
	
	// pick
	// double pick1[7] = {0.016432, 0.975207, 1.136231, -0.010454, -0.004100, -0.020581, 0.999725};
	double pick1[6] = {-0.115, 0.583, 1.299, -0.309, -1.579, -0.100};
	double pick_down = 0.13893 - 0.39687;
	double pick2[7] = {-1.0142, -0.057098, 0.39687, 0.0038908, -0.0015527, -0.0075026, 0.99996};

	// chuck
	double chuck1[6] = {1.323, -0.937, 2.272, -1.279, -1.832, -0.015};
	double chuck2[7] = {-0.21982, -0.86728, 0.50722, -0.48808, 0.48073, 0.51832, 0.51189};
	double chuck3[7] = {-0.14083, -0.86697, 0.50728, 0.70719, 0.03178, -0.70621, -0.011589};
	double chuck_insert = (0.074398) - (-0.14083);
	double chuck4[7];
	memcpy(chuck4, chuck3, sizeof(double)*7);
	double chuck5[7] = {-0.22875, -0.21745, 0.50815, -0.015327, 0.014137, 0.71178, 0.70209};

	// place
	double place1[6] = {-0.604, 0.163, 1.734, -0.322, -1.565, -0.606};
	double place2[7] = {-1.0526, 0.50414, 0.23745, -0.0036216, -6.914e-05, 0.0010558, 0.99999};
	double place_down = 0.15643 - 0.23745;
	double place3[7];
	memcpy(place3, place2, sizeof(double)*7);

	// camera pose
	// 0.16328, 0.78975, 1.2519, -0.019649, 0.69404, 0.0064938, 0.71964
	// -0.550, 0.439, 1.360, 1.278, -0.589, -3.069

	// geometry_msgs::Pose current_pose = move_group->getCurrentPose().pose;

	// ROS_INFO("current pose : %f, %f, %f, %f, %f, %f, %f", current_pose.position.x, current_pose.position.y, current_pose.position.z,
	// 	current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);

	Eigen::Vector3d Pc(-0.78765, 0.078789, 0.53742);
	// Eigen::Vector4d Qc(-0.0067942, 0.71963, -0.69405, 0.019816);
	// Eigen::Matrix3d Cc = quat2mat(Qc);
	// Eigen::Quaterniond Qc(0.019778, -0.0068241, 0.71963, -0.69404);
	Eigen::Quaterniond Qc(0.72018, 0.019246, 0.0064008, -0.69349);
	Eigen::Matrix3d Cc = Qc.toRotationMatrix();

	// current cam pose : 0.078848, 0.787577, 1.307499, -0.504048, -0.513677, 0.504755, 0.476753

	Eigen::Matrix3d Co, Cn;
	// Eigen::Matrix3d Cco;
	// Cco << 0.02242925132699292, -0.9933483061549334, -0.1129427790699121,
	// 	-0.9682068651796899, 0.006570538724136266, -0.250064580138396,
	// 	0.2491433220133107, 0.1149607353838649, -0.9616192772694258;

	// 	// 0.02077173700983082, -0.9930904968057878, -0.1154980523456063;
	// 	// -0.9690047285460736, 0.008446657992312701, -0.2468977319136637;
	// 	// 0.246167363793304, 0.1170466536163887, -0.9621339355205492
	// Eigen::Vector3d vco(2.043150891717752, -2.026699269469365, 0.1407238220564062);
	// Eigen::Matrix3d Ccn;
	// Ccn << 0.05608896767467619, -0.9813839750664258, -0.1836832087807966,
	// 	-0.9760775539166969, -0.01519067560784926, -0.2168913371127911,
	// 	0.2100634105340253, 0.1914542683185666, -0.958758899149007;
	// Eigen::Vector3d vcn(2.054386062180801, -1.980938587294669, 0.02669659598405302);

	// Eigen::Matrix3d temp, temp2;
	// temp << 0, 0, 1, 
	// 		-1, 0, 0, 
	// 		0, -1, 0;
	// temp2 << -1, 0, 0, 
	// 		0, -1, 0, 
	// 		0, 0, 1;
	// Co = Cc*temp*Cco;
	// Cn = Cc*Ccn;

	// std::cout << "cal : " << std::endl;
	// std::cout << Co << std::endl << std::endl;
	// std::cout << Cn << std::endl;

	Eigen::Vector3d Vo(-0.52995, -0.44017, 0.37001), Vn(-0.78936, -0.44893, 0.37001);
	Eigen::Quaterniond Qo(0.405529, 0.579279, -0.579219, 0.405571), Qn(-0.429119, -0.543504, 0.612913, -0.380524);
	Co = Qo.normalized().toRotationMatrix();
	Cn = Qn.normalized().toRotationMatrix();

	err_mat = Cn*Co.transpose();
	std::cout << "err mat : \n";
	std::cout << err_mat << std::endl;
	assert((err_mat * err_mat.transpose() - Eigen::Matrix3d::Identity()).norm() < 1e-6);\
	
	err_vec = Co*(Co.transpose()*Vo - Cn.transpose()*Vn);
	std::cout << std::endl << err_vec << std::endl << std::endl;

	ros::init(argc, argv, "robotplus_robot_control_node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	move_group = new moveit::planning_interface::MoveGroupInterface("arm");
	move_group->setPlanningTime(5.0);
	move_group->setMaxAccelerationScalingFactor(1.0);
	move_group->setMaxVelocityScalingFactor(0.5);
	move_group->setNumPlanningAttempts(10);
	move_group->setStartStateToCurrentState();

	move_group->setPoseReferenceFrame("base_link");
	// move_group->setEndEffectorLink("cam");
	// std::cout << move_group->getPlanningFrame() << std::endl;
	
	// geometry_msgs::Pose current_cam_pose = move_group->getCurrentPose().pose;
	// ROS_INFO("current cam pose : %f, %f, %f, %f, %f, %f, %f", current_cam_pose.position.x, current_cam_pose.position.y, current_cam_pose.position.z,
	// 	current_cam_pose.orientation.x, current_cam_pose.orientation.y, current_cam_pose.orientation.z, current_cam_pose.orientation.w);
	
	// exit(1);
#if 1
	// err_mat = Eigen::Matrix3d::Identity();
	// err_vec = Eigen::Vector3d::Zero();

	pthread_t t_logging;
	run = true;
	pthread_create(&t_logging, nullptr, logging, nullptr);

	movej(init_joint);

	movej(pick1);
	geometry_msgs::Pose current_pose = move_group->getCurrentPose().pose;
	ROS_INFO("current pose : %f, %f, %f, %f, %f, %f, %f", current_pose.position.x, current_pose.position.y, current_pose.position.z,
		current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
	// movel_base(pick2);
	pick2[2] += pick_down;
	movel_base(pick2);
	pick2[2] -= pick_down;
	movel_base(pick2);
	movej(init_joint);

	movej(chuck1);
	movel_machine(chuck2);
	movel_machine(chuck3);
	chuck4[0] += chuck_insert;
	movel_machine(chuck4);
	chuck4[0] -= chuck_insert;
	movel_machine(chuck4);
	movel_machine(chuck3);
	movel_machine(chuck2);
	movel_machine(chuck5);

	movej(place1);
	movel_base(place2);
	place3[2] += place_down;
	movel_base(place3);
	place3[2] -= place_down;
	movel_base(place3);
	movel_base(place2);
	movej(place1);

	movej(init_joint);

	run = false;
	pthread_join(t_logging, nullptr);

#else
	double cam[6] = {-0.550, 0.439, 1.360, 1.278, -0.589, -3.069};
	movej(cam);


#endif

	delete move_group;
	ros::shutdown();
	return 0;
}