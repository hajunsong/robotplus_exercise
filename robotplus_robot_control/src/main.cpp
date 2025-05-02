#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>

#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Dense>

moveit::planning_interface::MoveGroupInterface *move_group;
bool run = false;
Eigen::Matrix3d err_mat = Eigen::Matrix3d::Identity();
Eigen::Vector3d err_vec = Eigen::Vector3d::Zero();

Eigen::Vector4d mat2quat(Eigen::Matrix3d mat){
	double tr = mat(0, 0) + mat(1, 1) + mat(2, 2);
	double m00, m01, m02, m10, m11, m12, m20, m21, m22;
	double qw, qx, qy, qz;
	m00 = mat(0, 0); m01 = mat(0, 1); m02 = mat(0, 2);
	m10 = mat(1, 0); m11 = mat(1, 1); m12 = mat(1, 2);
	m20 = mat(2, 0); m21 = mat(2, 1); m22 = mat(2, 2);

	if (tr > 0) {
		double S = sqrt(tr + 1.0) * 2; // S=4*qw
		qw = 0.25 * S;
		qx = (m21 - m12) / S;
		qy = (m02 - m20) / S;
		qz = (m10 - m01) / S;
	}
	else if ((m00 > m11) & (m00 > m22)) {
		double S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx
		qw = (m21 - m12) / S;
		qx = 0.25 * S;
		qy = (m01 + m10) / S;
		qz = (m02 + m20) / S;
	}
	else if (m11 > m22) {
		double S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
		qw = (m02 - m20) / S;
		qx = (m01 + m10) / S;
		qy = 0.25 * S;
		qz = (m12 + m21) / S;
	}
	else {
		double S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
		qw = (m10 - m01) / S;
		qx = (m02 + m20) / S;
		qy = (m12 + m21) / S;
		qz = 0.25 * S;
	}

	Eigen::Vector4d quat(qx, qy, qz, qw);
	return quat;
}

Eigen::Matrix3d quat2mat(Eigen::Vector4d quat){
	Eigen::Matrix3d mat;

	double sqx = quat(0)*quat(0);
	double sqy = quat(1)*quat(1);
	double sqz = quat(2)*quat(2);
	double sqw = quat(3)*quat(3);

	// invs (inverse square length) is only required if quaternion is not already normalised
	double invs = 1 / (sqx + sqy + sqz + sqw);
	mat(0, 0) = ( sqx - sqy - sqz + sqw)*invs; // since sqw + sqx + sqy + sqz =1/invs*invs
	mat(1, 1) = (-sqx + sqy - sqz + sqw)*invs;
	mat(2, 2) = (-sqx - sqy + sqz + sqw)*invs;
	
	double tmp1 = quat(0)*quat(1);
	double tmp2 = quat(2)*quat(3);
	mat(1, 0) = 2.0 * (tmp1 + tmp2)*invs;
	mat(0, 1) = 2.0 * (tmp1 - tmp2)*invs;
	
	tmp1 = quat(0)*quat(2);
	tmp2 = quat(1)*quat(3);
	mat(2, 0) = 2.0 * (tmp1 - tmp2)*invs;
	mat(0, 2) = 2.0 * (tmp1 + tmp2)*invs;
	tmp1 = quat(1)*quat(2);
	tmp2 = quat(0)*quat(3);
	mat(2, 1) = 2.0 * (tmp1 + tmp2)*invs;
	mat(1, 2) = 2.0 * (tmp1 - tmp2)*invs;

	return mat;
}

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
	Eigen::Vector4d quat(cmd[3], cmd[4], cmd[5], cmd[6]);
	mat = quat2mat(quat);

	std::cout << "origin command : " << vec.transpose() << quat.transpose() << std::endl;

	Eigen::Matrix3d target_mat;
	Eigen::Vector3d target_vec;
	target_mat = mat*err_mat;
	target_vec = vec + err_vec;

	Eigen::Vector4d target_quat;
	target_quat = mat2quat(target_mat);

	std::cout << "correction command : " << target_vec.transpose() << target_quat.transpose() << std::endl;

	geometry_msgs::Pose target_pose;
	target_pose.position.x = target_vec(0);
	target_pose.position.y = target_vec(1);
	target_pose.position.z = target_vec(2);
	target_pose.orientation.x = target_quat(0);
	target_pose.orientation.y = target_quat(1);
	target_pose.orientation.z = target_quat(2);
	target_pose.orientation.w = target_quat(3);

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
	while (run)
	{
		geometry_msgs::Pose current_pose = move_group->getCurrentPose().pose;

		// Eigen::Vector4d quat(
		// 	current_pose.orientation.x,
		// 	current_pose.orientation.y,
		// 	current_pose.orientation.z,
		// 	current_pose.orientation.w);
		// Eigen::Matrix3d mat = quat2mat(quat);
		// Eigen::Vector3d vec(current_pose.position.x, current_pose.position.y, current_pose.position.z);

		// Eigen::Vector3d ref(0.58, 1.52, -0.77);
		// Eigen::Vector3d value = vec + ref;

		// fout << value(0) << ", ";
		// fout << value(1) << ", ";
		// fout << value(2) << ", ";
		// fout << quat(0) << ", ";
		// fout << quat(1) << ", ";
		// fout << quat(2) << ", ";
		// fout << quat(3) << ", ";
		// fout << "\n";

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
		
		ros::Duration(0.01).sleep();
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

	/*
	rmat : 
		[-0.01667032905351939, -0.9996741189203842, 0.01933277243441882;
		-0.9682836212379602, 0.01131989249562371, -0.2495970530198424;
		0.2492968691571354, -0.022880471905998, -0.9681567822588493]
	rvec : 
		[2.08138038104656; -2.111194324262539; 0.2881816835208755]


		 
	rmat : 
		[-0.01763863662019094, -0.9998443685631652, -0.0003422728540694031;
		-0.971190654610366, 0.01721451000175778, -0.2376812425137677;
		0.2376501438999562, -0.003859960870936846, -0.9713431472998638]
	rvec : 
		[2.0761773734942; -2.113214277401341; 0.2544259109239245]

	rmat : 
		[0.02218223345940262, -0.9959708212897407, -0.08689114833049828;
		-0.9739737036141656, -0.001918510018832809, -0.2266529152413748;
		0.225572988602046, 0.08965736143088146, -0.9700919463403422]
	rvec : 
		[2.072766652543755;
		-2.047563076240862;
		0.1441460974638388]
	*/

	Eigen::Vector3d Pe(0.1629, 0.83973, 1.2521);
	Eigen::Vector4d Qe(-0.019746, 0.69402, 0.0069196, 0.71965);
	Eigen::Matrix3d Ce = quat2mat(Qe);
	Eigen::Vector3d Pc(0.07888, 0.83765, 1.3074);
	Eigen::Vector4d Qc(0.50403, 0.51371, -0.50471, -0.47679);
	Eigen::Matrix3d Cc = quat2mat(Qc);
	Eigen::Vector3d Pec = Ce*(Pe - Pc);
	
	Eigen::Matrix3d Co;
	Co << 0.02079979022011202, -0.9931313380198488, -0.1151412791734756,
		-0.9688248701263298, 0.008415847203304017, -0.2476036036501706,
		0.2468719096039472, 0.1167018378550893, -0.9619952917191149;
	Eigen::Vector3d vo(2.041728447677992, -2.028881651911355, 0.1362241715515326);
	Eigen::Matrix3d Cn;
	Cn << 0.05684853735547113, -0.9802133262601278, -0.1896050601186586,
		-0.9754485223895604, -0.01405994482785733, -0.2197782931034389,
		0.2127637750383441, 0.1974440502357417, -0.9569469280257598;
	Eigen::Vector3d vn(2.049605114996181, -1.976637243471908, 0.02340710304943872);
	Eigen::Vector3d Vo(-0.52995, -0.44017, 0.37001), Vn(-0.57884, -0.37357, 0.37001);
	Eigen::Vector4d Qo(0.57928, -0.57922, 0.40557, 0.40553), Qn(-0.5435, 0.61291, -0.38052, -0.42912);
	// Eigen::Matrix3d Co, Cn;
	Co = quat2mat(Qo);
	Cn = quat2mat(Qn);

	double ang = 0.12;
	Eigen::Matrix3d temp_mat;
	err_mat << cos(ang), -sin(ang), 0, sin(ang), cos(0), 0, 0, 0, 1;
	
	err_mat = (Cn.transpose()*Co);
	err_vec = Cn*(Vo - Vn);

	std::cout << Cn << std::endl;
	std::cout << Co << std::endl;

	std::cout << err_mat << std::endl;
	std::cout << (Cn.transpose()*Co) << std::endl;

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
	std::cout << move_group->getPlanningFrame() << std::endl;

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