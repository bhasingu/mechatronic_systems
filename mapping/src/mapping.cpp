#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <chrono>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
using namespace std;

//variable for odom
double angular;
double linear;
double posX, posY, yaw;

#define maxlinearspeed 0.2
#define maxangularspeed pi/6

//variable for bumper/side
int BumperNumber = 0;
int SideNumber = 0;

//variables for laser
double midLaserRange_min = 0;
double midLaserRange = 0;
double leftLaserRange = 0;
double rightLaserRange = 0;
#define minSideLaser 0.5
#define minFrontLaser 0.6
int laserSize = 0, laserOffset = 0, desiredAngle = 10;
double pi = 3.1416;
double front;

//variables for angular tuning
int turncase = 0;
int stopturn = 0;

//limit the number of random turn
int limit = 0;

//robot state variable
enum CurrentState{
	ROTATE,
	STRAIGHT,
	WALLFOUND,
	PATHSEARCH,
	BUMPERHIT,
	SIDETOOCLOSE
};
ros::Time TotalRunTime;
ros::Time StateBegin; //Recording the start time for each state
CurrentState state = ROTATE;

void SetState(CurrentState stateNow){
	StateBegin = ros::Time::now();
	state = stateNow;
}

void bumperCallback(const kobuki_msgs::BumperEvent msg){
	//fill with your code
	if (msg.state == 0 || state == BUMPERHIT){
		return;
	}
	if (msg.bumper == 0)//Left Bumper
	{ 
		ROS_INFO("Left Bumper Hit!");
		BumperNumber = 1;
		SetState(BUMPERHIT);
	}
	else if (msg.bumper == 1)//Middle Bumper
	{ 
		ROS_INFO("Middle Bumper Hit!");
		SetState(WALLFOUND);
	}
	else if (msg.bumper == 2)//Right Bumper
	{ 
		ROS_INFO("Right Bumper Hit!");
		BumperNumber = -1;
		SetState(BUMPERHIT);
	}
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg){
	laserSize = (msg->angle_max - msg->angle_min) / msg->angle_increment;
	laserOffset = desiredAngle * pi / (180 * msg->angle_increment);
	midLaserRange_min = 11;
	midLaserRange = 11;
	leftLaserRange = 11;
	rightLaserRange = 11;
		for(int i = 0; i < laserSize/3; i++){
			if(leftLaserRange > msg->ranges[i])
				leftLaserRange = msg->ranges[i];
		}
		for(int i = 2*laserSize/3; i < laserSize; i++){
			if(rightLaserRange > msg->ranges[i])
				rightLaserRange = msg->ranges[i];
		}
		for(int i = laserSize/3; i < 2*laserSize/3; i++){
			if(midLaserRange_min > msg->ranges[i])
				midLaserRange_min = msg->ranges[i];
		}
	midLaserRange = msg->ranges[laserSize/2];
	if(midLaserRange_min == 11)
		midLaserRange_min = 0;
	if(midLaserRange == 11)
		midLaserRange = 0;
	if(rightLaserRange == 11)
		rightLaserRange = 0;
	if(leftLaserRange == 11)
		leftLaserRange = 0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);
}


double setdirection (double desired_yaw){
	ROS_INFO("set direction!");
	ros::spinOnce();
	double current_yaw = yaw;
	if (current_yaw * desired_yaw >= 0){
		if (desired_yaw >= current_yaw){
			turncase = 1;
			ROS_INFO("turncase = 1");
			return pi/6;
		}
		else if (desired_yaw < current_yaw){
			turncase = 2;
			ROS_INFO("turncase = 2");
			return -pi/6;
		}
	}
	else if (current_yaw * desired_yaw < 0){
		if (current_yaw>0 && desired_yaw<0){
			if ((abs(current_yaw)+abs(desired_yaw)>=pi)){
				turncase = 3;
				ROS_INFO("turncase = 3");
				return pi/6;
			}
			else if ((abs(current_yaw)+abs(desired_yaw)<pi)){
				turncase = 4;
				ROS_INFO("turncase = 4");
				return -pi/6;
			}
		}
		else if (current_yaw<0 && desired_yaw>0){
			if ((abs(current_yaw)+abs(desired_yaw))>=pi){
				turncase = 5;
				ROS_INFO("turncase = 5");
				return -pi/6;
			}
			else if ((abs(current_yaw)+abs(desired_yaw)<pi)){
				turncase = 6;
				ROS_INFO("turncase = 6");
				return pi/6;
			}
		}
	}
}

void stopturning(double desired_yaw){
	ROS_INFO("stopturning function entered!");
	ros::spinOnce();
	double current_yaw = yaw;
	if (turncase == 1){
		if (stopturn == 0){
			ros::spinOnce();
			current_yaw = yaw;
			ROS_INFO("turncase = 1, still turning");
			ROS_INFO("current_yaw: %f", current_yaw);
			if (current_yaw >= desired_yaw){
				stopturn = 1;
				ROS_INFO("done turning");
			}
		}
	}
	else if (turncase == 2){
		if (stopturn == 0){
			ros::spinOnce();
			current_yaw = yaw;
			ROS_INFO("turncase = 2, still turning");
			ROS_INFO("current_yaw: %f", current_yaw);
			if (current_yaw <= desired_yaw){
				stopturn = 1;
				ROS_INFO("done turning");
			}
		}
	}
	else if (turncase == 3){
		if (stopturn == 0){
			ros::spinOnce();
			current_yaw = yaw;
			ROS_INFO("turncase = 3, still turning");
			ROS_INFO("current_yaw: %f", current_yaw);
			if (current_yaw < 0 && current_yaw >= desired_yaw){
				stopturn = 1;
				ROS_INFO("done turning");
			}
		}
	}
	else if (turncase == 4){
		if (stopturn == 0){
			ros::spinOnce();
			current_yaw = yaw;
			ROS_INFO("turncase = 4, still turning");
			ROS_INFO("current_yaw: %f", current_yaw);
			if (current_yaw < 0 && current_yaw <= desired_yaw){
				stopturn = 1;
				ROS_INFO("done turning");
			}
		}
	}
	else if (turncase == 5){
		if (stopturn == 0){
			ros::spinOnce();
			current_yaw = yaw;
			ROS_INFO("turncase = 5, still turning");
			ROS_INFO("current_yaw: %f", current_yaw);
			if (current_yaw > 0 && current_yaw <= desired_yaw){
				stopturn = 1;
				ROS_INFO("done turning");
			}
		}
	}
	else if (turncase == 6){
		if (stopturn == 0){
			ros::spinOnce();
			current_yaw = yaw;
			ROS_INFO("turncase = 6, still turning");
			ROS_INFO("current_yaw: %f", current_yaw);
			if (current_yaw > 0 && current_yaw >= desired_yaw){
				stopturn = 1;
				ROS_INFO("done turning");
			}
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	//subscriber and publisher setup
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
	ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback); 
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	//angular = 0.0; //was local
	linear = 0.0;  //was local
	geometry_msgs::Twist vel;

	std::chrono::time_point<std::chrono::system_clock> start;
	start = std::chrono::system_clock::now(); /* start timer */
	uint64_t secondsElapsed = 0;			  // the timer just started, so we know it is less than 480, no need to check.

	ros::spinOnce();
	double yaw_old = yaw;
	double yaw_new = 0;
	double sum = 0;
	double difference;
	double max_distance = 0;
	double longyaw;
	double longrange;
	double degree = 0;
	bool first_turn = true;
	bool path_search = false;
	bool normal = false;
	bool do_path_search = false;

	while (ros::ok() && secondsElapsed <= 480)
	{
		ros::spinOnce();
		eStop.block();
		
		switch (state)
		{
		case ROTATE:
			ROS_INFO("CASE: ROTATE");
			ros::spinOnce();
			difference = 0;
			longrange = 0;
			longyaw = 0;
			degree = 0; // could cause problem
			linear = 0;
			ROS_INFO("degree: %f", degree);
			yaw_old = yaw;
			while (degree < 2 * pi){
				angular = pi/6;
				vel.angular.z = angular;
				vel.linear.x = linear;
				vel_pub.publish(vel);
				ros::spinOnce();

				// Implementing Gurtej's turn
				yaw_new=yaw;
				if (yaw_new*yaw_old < 0){
					if (yaw_new < 0){
						difference = 2*pi - yaw_old - abs(yaw_new);
					}
					else if (yaw_new > 0){ // added this for the case of angular = -pi/6
						difference = yaw_new + abs(yaw_old); 
					}
					degree = degree + difference;
					yaw_old = yaw_new;
				}
				else if (yaw_new*yaw_old >= 0){
					difference = yaw_new - yaw_old;
					degree = difference + degree;
					yaw_old = yaw_new;
				}
				if (midLaserRange >= longrange){
                    if (path_search && limit < 7){
                        if (front > 1.0 && midLaserRange > 1.5 && (((pi/4 <= degree) && (degree <= pi/2)) || (((3*pi)/2 <= degree) && (degree < (7*pi)/4)))){
                            longyaw=yaw;
                            longrange = midLaserRange;
							do_path_search = true;
                        }
                    }
					else if (first_turn){
						longyaw = yaw;
						longrange = midLaserRange;
					}
					else if (normal && (degree <= pi/2 || degree >= (3*pi)/2)){
						longyaw = yaw;
						longrange = midLaserRange;
					}
				}
				ROS_INFO("degree: %f", degree);

				if (degree >= 2*pi){
					angular = 0;
					vel.angular.z = angular;
					vel_pub.publish (vel);
					ROS_INFO("rotation done");
					// first_turn = false;
                    // path_search = false;
				}
			}
			if (do_path_search) limit = limit + 1;
			if (do_path_search || first_turn || normal){
				// Tuning to the desired direction
				angular = setdirection(longyaw);
				while (stopturn == 0){ //change this variable to boolean
					ROS_INFO("longyaw: %f", longyaw);
					ROS_INFO("angular: %f", angular);
					stopturning(longyaw);
					vel.angular.z = angular;
					vel_pub.publish(vel);
				}
			}
			stopturn = 0;
			SetState(STRAIGHT);
			first_turn = false;
			break;

		case STRAIGHT:
			ROS_INFO("CASE: STRAIGHT");
			angular = 0;
			ros::spinOnce();
			if(midLaserRange <= minFrontLaser){
				if (linear > 0.05){
					linear = linear - 0.001;
					break;
				}
				SetState(ROTATE);
				normal = true;
				path_search = false;
				do_path_search = false;
				break;
			}
			if(rightLaserRange <= minSideLaser){
				if (linear > 0.05){
					linear = linear - 0.001;
					break;
				}
				path_search = false;
				normal = false;
				do_path_search = false;
				linear = 0.05;
				ROS_INFO("Right Too Close!");
				SideNumber = -1;
				SetState(SIDETOOCLOSE);
				break;
			}
			if(leftLaserRange <= minSideLaser){
				if (linear > 0.05){
					linear = linear - 0.001;
					break;
				}
				linear = 0.05;
				ROS_INFO("Left Too Close!");
				path_search = false;
				normal = false;
				do_path_search = false;
				SideNumber = 1;
				SetState(SIDETOOCLOSE);
				break;
			}
			if (midLaserRange_min < 0.7 || rightLaserRange < 0.85 || leftLaserRange < 0.85){
				if (linear > 0.15){
					linear = linear - 0.001;
					break;
				}
				path_search = false;
				normal = false;
				do_path_search = false;
			}
			if((ros::Time::now() - StateBegin) > ros::Duration(8.0)/* && limit < 5*/){
				SetState(PATHSEARCH);
				normal = false;
				do_path_search = false;
				break;
			}
			if (linear < maxlinearspeed){
				linear = linear + 0.001;
				break;
			}
			path_search = false;
			normal = false;
			do_path_search = false;
			break;


		case BUMPERHIT:
			ROS_INFO("CASE: BUMPERHIT");
			if (ros::Time::now() - StateBegin > ros::Duration(1))
			{
				SetState(STRAIGHT);
				linear = 0;
				angular = 0;
				break; 
			}
			linear = -1*maxlinearspeed; // move back and turn away from bumper hit
			angular = -1*BumperNumber*maxangularspeed;
			
			break;

		case SIDETOOCLOSE:
			ROS_INFO("CASE: SIDETOOCLOSE");
			if (ros::Time::now() - StateBegin > ros::Duration(1))
			{
				SetState(STRAIGHT);
				break; 
			}
			linear = 0.05; // move back and turn away from bumper hit

			if (SideNumber == 1 && angular < SideNumber*maxangularspeed){
				angular = angular + 0.01;
				break;
			}
			if (SideNumber == -1 && abs(angular) < abs(SideNumber*maxangularspeed)){
				angular = angular - 0.01;
				break;
			}			
			break;



		case WALLFOUND:
			ROS_INFO ("wallfound!");
			if(leftLaserRange > rightLaserRange)
			{
				BumperNumber = -1;
				SetState(BUMPERHIT);
			}
			else if(rightLaserRange > leftLaserRange)
			{
				BumperNumber = 1;
				SetState(BUMPERHIT);
			}
			break;
			

		case PATHSEARCH:	
			ROS_INFO("CASE: PATHSEARCH");
			ros::spinOnce();
			front = midLaserRange;
			normal = false;	
            path_search = true;
			do_path_search = false;
			linear = 0;
			// limit = limit + 1;
            SetState(ROTATE);
            break;
		}

		// The last thing to do is to update the timer.
		vel.angular.z = angular;
		vel.linear.x = linear;
		vel_pub.publish(vel);
		ros::spinOnce();
		ros::Duration(0.01).sleep();

		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
	}
	return 0;
}

