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

	std::cout << "ori command : " << vec.transpose() << " " << quat.transpose() << std::endl;

	Eigen::Matrix3d target_mat;
	Eigen::Vector3d target_vec;
	target_mat = err_mat.transpose()*mat;
	target_vec = err_mat.transpose()*vec;

	Eigen::Vector4d target_quat;
	target_quat = mat2quat(target_mat);

	std::cout << "cor command : " << target_vec.transpose() << " "  << target_quat.transpose() << std::endl;

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
	fout.open("/home/keti/logging_cor.txt");

	std::ofstream fout_joint;
	fout_joint.open("/home/keti/logging_cor_jnt.txt");
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
	Eigen::Vector4d Qc(-0.0067942, 0.71963, -0.69405, 0.019816);
	Eigen::Matrix3d Cc = quat2mat(Qc);

	// current cam pose : 0.078848, 0.787577, 1.307499, -0.504048, -0.513677, 0.504755, 0.476753

	Eigen::Matrix3d Co, Cn;
	Eigen::Matrix3d Cco;
	Cco << 0.02242925132699292, -0.9933483061549334, -0.1129427790699121,
		-0.9682068651796899, 0.006570538724136266, -0.250064580138396,
		0.2491433220133107, 0.1149607353838649, -0.9616192772694258;

		// 0.02077173700983082, -0.9930904968057878, -0.1154980523456063;
		// -0.9690047285460736, 0.008446657992312701, -0.2468977319136637;
		// 0.246167363793304, 0.1170466536163887, -0.9621339355205492
	Eigen::Vector3d vco(2.043150891717752, -2.026699269469365, 0.1407238220564062);
	Eigen::Matrix3d Ccn;
	Ccn << 0.05608896767467619, -0.9813839750664258, -0.1836832087807966,
		-0.9760775539166969, -0.01519067560784926, -0.2168913371127911,
		0.2100634105340253, 0.1914542683185666, -0.958758899149007;
	Eigen::Vector3d vcn(2.054386062180801, -1.980938587294669, 0.02669659598405302);

	Co = Cc*Cco;
	Cn = Cc*Ccn;

	std::cout << "cal : " << std::endl;
	std::cout << Co << std::endl << std::endl;
	std::cout << Cn << std::endl;

	Eigen::Vector3d Vo(-0.52995, -0.44017, 0.37001), Vn(-0.57884, -0.37357, 0.37001);
	Eigen::Vector4d Qo(0.57928, -0.57922, 0.40557, 0.40553), Qn(-0.5435, 0.61291, -0.38052, -0.42912);
	Co = quat2mat(Qo);
	Cn = quat2mat(Qn);

	std::cout << "ans : " << std::endl;
	std::cout << Co << std::endl << std::endl;
	std::cout << Cn << std::endl;

	// double ang = 0.12;
	// Eigen::Matrix3d temp_mat;
	// err_mat << cos(ang), -sin(ang), 0, sin(ang), cos(ang), 0, 0, 0, 1;
	
	err_mat = (Cn.transpose()*Co);
	err_vec = (Vo - Vn);

	// std::cout << "ans : " << std::endl;
	// std::cout << err_mat << std::endl;
	// std::cout << "cal : " << std::endl;
	// std::cout << Co*Cn.transpose() << std::endl;

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
	move_group->setEndEffectorLink("cam");
	// std::cout << move_group->getPlanningFrame() << std::endl;
	
	// geometry_msgs::Pose current_cam_pose = move_group->getCurrentPose().pose;
	// ROS_INFO("current cam pose : %f, %f, %f, %f, %f, %f, %f", current_cam_pose.position.x, current_cam_pose.position.y, current_cam_pose.position.z,
	// 	current_cam_pose.orientation.x, current_cam_pose.orientation.y, current_cam_pose.orientation.z, current_cam_pose.orientation.w);

	

#if 0
	// err_mat = Eigen::Matrix3d::Identity();
	err_vec = Eigen::Vector3d::Zero();

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