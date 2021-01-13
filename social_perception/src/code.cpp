#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <cmath>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include "ros/time.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <imagePipeline.h>

using namespace cv;
using namespace std;

std::string dir_im = "/home/jok/catkin_ws/src/mie443_contest3/images/";
std::string dir_so = "/home/jok/catkin_ws/src/mie443_contest3/sounds/";

geometry_msgs::Twist follow_cmd;
ros::Time g_startTime;
int world_state=0;
double pi = 3.1416;
bool dropped = false;
bool play = false;
int template_id;
bool lost_left = false;
bool lost_right = false;

void wheelDropCB(const kobuki_msgs::WheelDropEvent msg){
    if(msg.state == kobuki_msgs::WheelDropEvent::DROPPED){
        ROS_INFO("BOT LIFTED");
        dropped = true;
		play = false;
		world_state = 3;
		std::cout<<"lifted"<<msg.state<<std::endl;
    }
	else if(msg.state == kobuki_msgs::WheelDropEvent::RAISED){
		ROS_INFO("ROBOT ON GROUND");
		play = true;
		world_state = 3;
		std::cout<<"On the Ground"<<msg.state<<std::endl;
	}
}

void followerCB(const geometry_msgs::Twist msg){
	follow_cmd = msg;
	std::cout<<"wheel drop"<<dropped<<std::endl;
	if (follow_cmd.linear.x == 0 && follow_cmd.angular.z == 0){
        if (!dropped){
            world_state = 1;
            if(ros::Time::now() - g_startTime > ros::Duration(10)){
                world_state = 5;
            }
        }
	}
}

void bumperCB(const kobuki_msgs::BumperEvent msg){
	if (!dropped && msg.bumper == 1){
        ROS_INFO("Bumper Hit!");
        world_state = 2;
    }
	if (!dropped && msg.bumper == 0){
		ROS_INFO("lost_left");
		lost_left = true;
	}
	if (!dropped && msg.bumper == 2){
		ROS_INFO("lost_right");
		lost_right == true;
	}
}

void showImage(std::string image_name, double dur = 2) {
	Mat image = imread(image_name.c_str(), CV_LOAD_IMAGE_COLOR);
	cv::imshow( "Images", image);
	cv::waitKey(1);
	ros::Duration(dur).sleep();
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	//start sound
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;
	ImagePipeline imagePipeline(nh);

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber lifter = nh.subscribe("mobile_base/events/wheel_drop",10,&wheelDropCB);
	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	ros::Duration(0.5).sleep();
	cv::Mat image;
	sc.playWave(dir_so + "mario1.wav");


	while(ros::ok()){
        if (world_state != 3){
            world_state = 0;
        }
		// template_id = imagePipeline.getTemplateID();
		// if (template_id == 1 && world_state != 3){
		// 	world_state = 4;
		// }

		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................
		std::cout << "world state:" << world_state << std::endl;
		
		if(!dropped && world_state == 0){
			ROS_INFO("Following");
			showImage(dir_im + "following.jpg", 0);
            // cv::destroyAllWindows();
			vel_pub.publish(follow_cmd);								
			g_startTime = ros::Time::now();

		}

		if(world_state == 1){ // For losing person
			sc.stopWave(dir_so + "mario1.wav");
			ROS_INFO("Person lost!");
			vel.angular.z = 0;
            vel.linear.x = 0;
			vel_pub.publish(vel);
			sc.playWave(dir_so + "sad.wav");
			showImage(dir_im + "sad.jpg", 0);
			sc.stopWave(dir_so + "sad.wav");
			sc.playWave(dir_so + "mario1.wav");
		}

		if(world_state == 2){ // For bumperhit
			sc.stopWave(dir_so + "mario1.wav");
			vel.angular.z = 0;
            sc.playWave(dir_so + "surprise.wav");
			showImage(dir_im + "surprise.jpg", 0);
			vel.linear.x = -0.2;
			vel_pub.publish(vel);
			ros::Duration(0.05).sleep();
			vel.linear.x = 0.2;
			vel_pub.publish(vel);
			ros::Duration(0.05).sleep();
			vel.linear.x = 0;
			vel_pub.publish(vel);
            ros::Duration(0.05).sleep();
			vel.linear.x = -0.2;
            vel_pub.publish(vel);
			ros::Duration(2).sleep();
            vel.linear.x = 0;
			vel_pub.publish(vel);
            ROS_INFO("Bumper hit!");
			sc.stopWave(dir_so + "surprise.wav");
			sc.playWave(dir_so + "mario1.wav");	
		}

		if(world_state == 3){ // For wheel dropped
			if (play){
				sc.playWave(dir_so + "mario1.wav");
				world_state = 0;
                play = false;
                dropped = false;
				continue;
			}
			sc.stopWave(dir_so + "mario1.wav");
			ROS_INFO("Wheel dropped!");
			vel.angular.z = 0;
            vel.linear.x = 0;
			vel_pub.publish(vel);
			sc.playWave(dir_so + "fear.wav");
			showImage(dir_im + "fear.jpg");
			ros::Duration(2).sleep();
			sc.stopWave(dir_so + "fear.wav");
		}

		// if(world_state == 4){ // see images
		// 	sc.stopWave(dir_so + "mario1.wav");
		// 	ROS_INFO("See Food!");
        //     sc.playWave(dir_so + "positively_excited.wav");
        //     showImage(dir_im + "positively_excited.jpg", 0);
		// 	while(imagePipeline.getTemplateID() == 1){
		// 		while((ros::Time::now()-g_startTime) < ros::Duration(2)){
		// 			vel.linear.x = 0.2;
		// 			vel.angular.z = 0;
		// 			vel_pub.publish(vel);
		// 		}
		// 		ros::spinOnce();
		// 	}
        //     sc.stopWave(dir_so + "positively_excited.wav");
		// 	sc.playWave(dir_so + "mario1.wav");				
		// }

		if(world_state == 5){ // Lost for 10 seconds
			ROS_INFO("Lost for 10 seconds!");
			sc.stopWave(dir_so + "mario1.wav");
            sc.playWave(dir_so + "anger.wav");
            showImage(dir_im + "anger.jpg",0);
			for(int i = 0; i<2 ; i++){
				vel.angular.z = 0;
				vel.linear.x = -0.2;
				vel_pub.publish(vel);
				ros::Duration(0.05).sleep();
				vel.linear.x = 0.2;
				vel_pub.publish(vel);
				ros::Duration(0.05).sleep();
				vel.linear.x = 0;
				vel_pub.publish(vel);
				ros::Duration(0.05).sleep();
				vel.angular.z = pi/2;
				vel_pub.publish(vel);
				ros::Duration(1).sleep();
				vel.angular.z = -pi/2;
				vel_pub.publish(vel);
				ros::Duration(2).sleep();
				ros::spinOnce();
			}
			sc.stopWave(dir_so + "anger.wav");
			sc.playWave(dir_so + "mario1.wav");	
			if (lost_left){
				vel.angular.z = pi/4;
            	vel_pub.publish(vel);
				ros::Duration(2).sleep();
                lost_left = false;
				continue;
			}
			if (lost_right){
				vel.angular.z = -pi/4;
            	vel_pub.publish(vel);
				ros::Duration(2).sleep();
                lost_right = false;
				continue;
			}	
		}
		ros::Duration(0.05).sleep();
	}
	return 0;
}