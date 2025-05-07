#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_control_validation_node");

	ros::NodeHandle nh;
	// ros::AsyncSpinner spinner(1);
	// spinner.start();

	ros::Publisher pub1 = nh.advertise<sensor_msgs::JointState>("/ori/joint_states", 1);
	ros::Publisher pub2 = nh.advertise<sensor_msgs::JointState>("/cor/joint_states", 1);

    std::vector<double> joint1, joint2;
    joint1.assign(7, 0);
    joint2.assign(7, 0);
    
    sensor_msgs::JointState msg1, msg2;

    msg1.name.push_back("q1");
	msg1.name.push_back("q2");
	msg1.name.push_back("q3");
	msg1.name.push_back("q4");
	msg1.name.push_back("q5");
	msg1.name.push_back("q6");
	msg1.position.assign(6, 0);
	msg1.velocity.assign(6, 0);
	msg1.effort.assign(6, 0);
    
    msg2.name.push_back("q21");
	msg2.name.push_back("q22");
	msg2.name.push_back("q23");
	msg2.name.push_back("q24");
	msg2.name.push_back("q25");
	msg2.name.push_back("q26");
	msg2.position.assign(6, 0);
	msg2.velocity.assign(6, 0);
	msg2.effort.assign(6, 0);

    std::ifstream read1;
    read1.open("/home/keti/logging_ori_jnt.txt");
    std::string line1;
    
    std::ifstream read2;
    read2.open("/home/keti/logging_cor_jnt.txt");
    std::string line2;

    while(ros::ok()){
        getline(read1, line1);
        std::stringstream ss1(line1);
        std::string value1;
        // while(getline(ss1, value1, ','))
        for(int i = 0; i < 7; i++)
        {
            getline(ss1, value1, ',');
            joint1[i] = atof(value1.c_str());
        }

        getline(read2, line2);
        std::stringstream ss2(line2);
        std::string value2;
        for(int i = 0; i < 7; i++)
        {
            getline(ss2, value2, ',');
            joint2[i] = atof(value2.c_str());
        }

        msg1.header.stamp = ros::Time::now();
        for(int i = 0; i < 6; i++){
            msg1.position[i] = joint1[i + 1];
        }
        pub1.publish(msg1);
        
        msg2.header.stamp = ros::Time::now();
        for(int i = 0; i < 6; i++){
            msg2.position[i] = joint2[i + 1];
        }
        pub2.publish(msg2);

        if(line1.size() == 0 && line2.size() == 0){
            read1.close();
            read2.close();
            break;
        }

		ros::Duration(0.01).sleep();
    }

    msg1.header.stamp = ros::Time::now();
    msg2.header.stamp = ros::Time::now();
    
    for(int i = 0; i < 6; i++) msg1.position[i] = 0;
    for(int i = 0; i < 6; i++) msg2.position[i] = 0;

    pub1.publish(msg1);
    pub2.publish(msg2);
    
	ros::shutdown();
	return 0;
}