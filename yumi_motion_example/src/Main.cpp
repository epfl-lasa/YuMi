#include <iostream>
#include <assert.h>
#include <string>
#include "eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include  "std_msgs/Float64.h"
#define JOINT_NUMBER 14


using namespace std;
using namespace Eigen;

bool Joint_state_recived=false;
VectorXd State_handle;
VectorXd State;
VectorXd State_left;
VectorXd State_right;
VectorXd joint_positions_desired_left;
VectorXd joint_positions_desired_right;
string Name_joint[14];
trajectory_msgs::JointTrajectory armCommand;
std_msgs::Float64 desiredConfiguration_left[7];
std_msgs::Float64 desiredConfiguration_right[7];

float joint_positions_down[JOINT_NUMBER] = {-1.5901633501052856, 2.758988380432129, -0.776835024356842, -0.7566481828689575, -1.2274471521377563, -1.1534804105758667, -3.3565382957458496, -3.377692699432373, 1.941343903541565, 1.8748698234558105, 1.8207812309265137, 1.2813187837600708, -0.5231171250343323, -0.42509302496910095};
float joint_positions_up[JOINT_NUMBER] = {-0.004894050769507885, 0.01121285930275917, -0.7481237053871155, -0.7881973385810852, 0.39989161491394043, 0.45870086550712585, -1.0020086765289307, -2.4347167015075684, 0.9538304805755615, -0.7194674015045166, 0.5497622489929199, -0.2827472388744354, 0.15741610527038574, 0.007038898766040802};
float joint_positions_zero[JOINT_NUMBER] = {0, 0,0,0, 0.39989161491394043, 0.45870086550712585, -1.0020086765289307, -2.4347167015075684, 0.9538304805755615, -0.7194674015045166, 0.5497622489929199, -0.2827472388744354, 0.15741610527038574, 0.007038898766040802};

void chatterCallback_state(const sensor_msgs::JointState & msg)
{
	if (msg.position.size()==JOINT_NUMBER)
	{
		for (int i=0;i<JOINT_NUMBER;i++)
		{
			if (Joint_state_recived==false)
			{
				State_handle(i)=msg.position[i];
			}
			State(i)=msg.position[i];
		}
		for (int i=0;i<7;i++)
		{
			State_left(i)=msg.position[2*i];
			State_right(i)=msg.position[2*i+1];
		} 
		if (Joint_state_recived==false)
		{
			for (int i=0;i<14;i++)
			{
				Name_joint[i]=msg.name[i];
			}

		}
	}

	Joint_state_recived=true;
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "yumi_example");
	ros::NodeHandle n;
	ros::Publisher armJointTrajectoryPublisher_left[7];
	ros::Publisher armJointTrajectoryPublisher_right[7];
	ros::Subscriber armJointTrajectorysub;


	armJointTrajectorysub = n.subscribe("/yumi/joint_states", 3, chatterCallback_state);

	armJointTrajectoryPublisher_left[0] = n.advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_1_l/command", 1);
	armJointTrajectoryPublisher_left[1] = n.advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_2_l/command", 1);
	armJointTrajectoryPublisher_left[2] = n.advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_3_l/command", 1);
	armJointTrajectoryPublisher_left[3] = n.advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_4_l/command", 1);
	armJointTrajectoryPublisher_left[4] = n.advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_5_l/command", 1);
	armJointTrajectoryPublisher_left[5] = n.advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_6_l/command", 1);
	armJointTrajectoryPublisher_left[6] = n.advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_7_l/command", 1);

	armJointTrajectoryPublisher_right[0] = n.advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_1_r/command", 1);
	armJointTrajectoryPublisher_right[1] = n.advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_2_r/command", 1);
	armJointTrajectoryPublisher_right[2] = n.advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_3_r/command", 1);
	armJointTrajectoryPublisher_right[3] = n.advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_4_r/command", 1);
	armJointTrajectoryPublisher_right[4] = n.advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_5_r/command", 1);
	armJointTrajectoryPublisher_right[5] = n.advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_6_r/command", 1);
	armJointTrajectoryPublisher_right[6] = n.advertise<std_msgs::Float64 > ("/yumi/joint_vel_controller_7_r/command", 1);


	State_handle.resize(14);
	State.resize(14);
	State_left.resize(7);
	State_right.resize(7);
	joint_positions_desired_right.resize(7);
	joint_positions_desired_left.resize(7);
	//desiredConfiguration.velocities.resize(JOINT_NUMBER);  //5 arm joints + 1 gripper joint
	//desiredConfiguration.positions.resize(JOINT_NUMBER);  //5 arm joints + 1 gripper joint
	//desiredConfiguration.accelerations.resize(JOINT_NUMBER);  //5 arm joints + 1 gripper joint
	armCommand.joint_names.resize(JOINT_NUMBER);

	ros::Rate rate(50); //Hz

	for(int i=0;i<7;i++)
	{
		desiredConfiguration_left[i].data=0.0;
		desiredConfiguration_right[i].data=0.0;
		joint_positions_desired_left(i)=0.0;
		joint_positions_desired_right(i)=0.0;
	}


	while  (Joint_state_recived==false)
	{
		ros::spinOnce();
		cout<<"Waiting for the state of the robot."<<endl;
	}

	cout<<"The position is recieved"<<endl;
	for (int i = 0; i < JOINT_NUMBER; ++i) {
		armCommand.joint_names[i] = Name_joint[i];
	};


	while (ros::ok())
	{
		ros::spinOnce();
//		cout<<"-------"<<endl;
//		cout<<"ERROR OF State of the robot ";
//		cout<<" ERROR "<<(State_handle-State).norm()<<" ";
		for (int i=0;i<7;i++)
		{	
			desiredConfiguration_left[i].data =-0.2*(State_left(i)-joint_positions_desired_left(i));
			desiredConfiguration_right[i].data =-0.2*(State_left(i)-joint_positions_desired_right(i));
		}
	//	for (int i=0;i<JOINT_NUMBER;i++)
	//	{
	//		desiredConfiguration.positions[i] =(joint_positions_up[i]- State(i))*0.2+State(i);
	//		desiredConfiguration.positions[i] =joint_positions_up[i];
	//		desiredConfiguration.positions[i] =State(i)-0.1;
		//	desiredConfiguration.velocities[i] =(joint_positions_up[i]- State(i));
	//		desiredConfiguration.positions[i] =joint_positions_down[i];
	//		State_handle(i)=desiredConfiguration.velocities[i]+State(i);
		//	desiredConfiguration.positions[i] = joint_positions_down[i];
	//	}
	//	desiredConfiguration.velocities[12] =(joint_positions_up[12]- State(12))/100;
		//desiredConfiguration.velocities[13] =(joint_positions_up[13]- State(13))/100;
	//	cout<<"-------"<<endl;
//		armCommand.header.stamp = ros::Time::now();
//		armCommand.header.frame_id = "yumi_body";
//		armCommand.points.resize(1);
//		armCommand.points[0] = desiredConfiguration;
 //      	armCommand.points[0].time_from_start = ros::Duration(0.01); // 1 ns

   //    	armJointTrajectoryPublisher.publish(armCommand);
		for (int i=0;i<7;i++)
		{	
			armJointTrajectoryPublisher_left[i].publish(desiredConfiguration_left[i]);
		//	armJointTrajectoryPublisher_right[i].publish(desiredConfiguration_right[i]);
		}
		rate.sleep();
	}

	return 0;
}
