#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <math.h>

#include <iostream>
#include <chrono>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#define _USE_MATH_DEFINES

using namespace std;

//Bumper Global Variables
bool bumperLeft = 0, bumperCenter = 0, bumperRight = 0;
bool bumperLeftIndex = 0, bumperCenterIndex = 0, bumperRightIndex = 0;

//LASER - Kinect Global Variables
double frontDist = 10, midDist = 10;
double leftDist = 10, rightDist = 10;
double left5Dist = 10, right5Dist = 10;
double leftDist_last = 10, rightDist_last = 10;
double left5Dist_last = 10, right5Dist_last = 10;
double sideAngleDiff = 0, side5tomidAngle = 0;
int laserSize = 0, laserOffset = 0, desiredFrontAngle = 5;

//Odom Global Variables
double posX, posY, yaw;
double pi = 3.1415926;


void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	if(msg->bumper == 0){
		bumperLeft = !bumperLeft;
		bumperLeftIndex = 1;}
	else if(msg->bumper == 1){
		bumperCenter = !bumperCenter;
		bumperCenterIndex = 1;}
	else if(msg->bumper == 2){
		bumperRight = !bumperRight;
		bumperRightIndex = 1;}
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){ 
	//Note: angle_max, angle_min and angle_increment are the parameters of the sensors and by default the unit is radian.
	//The Microsoft Kinect 360 cemera tilting range: 43 degree vertical x 57 degree horizontal
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment; //Laser array size
	laserOffset = desiredFrontAngle*pi/(180*msg->angle_increment);
	frontDist = 0;
	int frontDistCount = 0;

	//ROS_INFO("Size of laser scan array: %i and size of offset: %i", laserSize, laserOffset);
	
	//Taking the average of laser readings from the front 10 degree angle as frontDist
	if(desiredFrontAngle*pi/180 < msg->angle_max && -desiredFrontAngle*pi/180 > msg->angle_min){
		for(int i =laserSize/2 - laserOffset; i < laserSize/2 + laserOffset; i++){
			if(!isnan(msg->ranges[i]) && msg->ranges[i] != 11)
				frontDist += msg->ranges[i];
				frontDistCount ++;
		}
	}
	frontDist = frontDist/frontDistCount;

	//Reading Error Cases
	if(frontDist == 11) frontDist = 0.9;
	if(isinf(frontDist)) frontDist = 0;

	//Read left and right distances, at +/- 30 degree and +/- 25 degree
	if (!isnan(msg->ranges[6]))
		rightDist = msg->ranges[6];
	else
		rightDist = 0;

	if (!isnan(msg->ranges[laserSize-7]))
		leftDist = msg->ranges[laserSize-7];
	else
		leftDist = 0;

	if (!isnan(msg->ranges[55]))
		right5Dist = msg->ranges[55];
	else
		right5Dist = 0;

	if (!isnan(msg->ranges[laserSize-56]))
		left5Dist = msg->ranges[laserSize-56];
	else
		left5Dist = 0;

	//Calculate angle difference between sideDist and side5Dist
	sideAngleDiff = 49*msg->angle_increment;
	side5tomidAngle = (laserSize/2 - 55)*msg->angle_increment;

	//Read the most middle laser reading as midDist, specificly used as the criteria for randomPath function
	if (!isnan(msg->ranges[laserSize/2]) && msg->ranges[laserSize/2] != 11)
		midDist = msg->ranges[laserSize/2];
	else
		midDist = 0;

}


void odomCallback (const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	//ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, yaw*180/pi);

}


void publishTwist(double linear_vel, double angular_vel, ros::Publisher vel_pub){
	
	geometry_msgs::Twist vel;

	vel.linear.x = linear_vel;
	vel.linear.y = 0;
	vel.linear.z = 0;
	vel.angular.x = 0;
	vel.angular.y = 0;
	vel.angular.z = angular_vel;

	vel_pub.publish(vel);
}


void rotateAngle(double angularSpeed, double angleToRotate, ros::Publisher vel_pub){
	/* This function is used to rotate the robot a certain angle (rad) */
	/* angleToRotate +ve means rotate CCW, -ve means rotate CW */
	ros::spinOnce();
	double theta = yaw; //initialize original direction
	ROS_INFO("rotateAngle: %f", angleToRotate);

	bool bumperIndex = 0; //If during the rotation, any bumper is activated, stop rotate

	if(angleToRotate > 0){
		if(theta+angleToRotate <= pi){
			if(theta>-pi/6){
				while(yaw < theta+angleToRotate && yaw >= -pi/6 && !bumperIndex){
					publishTwist(0, angularSpeed, vel_pub);
					ros::spinOnce();
					if(bumperRight || bumperLeft || bumperCenter)
						bumperIndex = 1;
				}
			}
			else{
				while(yaw < theta+angleToRotate && !bumperIndex){
					publishTwist(0, angularSpeed, vel_pub);
					ros::spinOnce();
					if(bumperRight || bumperLeft || bumperCenter)
						bumperIndex = 1;
				}
			}
		}
		else{
			while((yaw > 0 || yaw < (-pi*2+angleToRotate+theta)) && !bumperIndex){
				//ROS_INFO("yaw: %f", yaw);
				publishTwist(0, angularSpeed, vel_pub);
				ros::spinOnce();
				if(bumperRight || bumperLeft || bumperCenter)
					bumperIndex = 1;
			}
		}
	}
	else if(angleToRotate < 0){
		if(theta+angleToRotate > -pi){
			if(theta < pi/6){
				while(yaw > theta+angleToRotate && yaw <= pi/6 && !bumperIndex){
					publishTwist(0, -angularSpeed, vel_pub);
					ros::spinOnce();
					if(bumperRight || bumperLeft || bumperCenter)
						bumperIndex = 1;
				}
			}
			else{
				while(yaw > theta+angleToRotate && !bumperIndex){
					publishTwist(0, -angularSpeed, vel_pub);
					ros::spinOnce();
					if(bumperRight || bumperLeft || bumperCenter)
						bumperIndex = 1;
				}
			}
		}
		else{
			while((yaw < 0 || yaw > (pi*2+angleToRotate+theta)) && !bumperIndex){
				publishTwist(0, -angularSpeed, vel_pub);
				ros::spinOnce();
				if(bumperRight || bumperLeft || bumperCenter)
					bumperIndex = 1;
			}
		}
	}
	else{
		publishTwist(0, 0, vel_pub);
		ros::spinOnce();
	}

	ROS_INFO("The robot rotated");
	publishTwist(0, 0, vel_pub);
}


void driveStraight(double speed, double distance, ros::Publisher vel_pub){
	/* This function is used to drive the robot forward a certain distance */
	ros::spinOnce();
	//initialize x y and yaw position from odom
	double x = posX;
	double y = posY;

	ROS_INFO("driveStraight: %f", distance);

	double distanceMoved = sqrt(pow(posX-x,2) + pow(posY-y,2));
	
	bool bumperIndex = 0; //If during the movement, any bumper is activated, stop

	while(distanceMoved < distance && !bumperIndex){
		ros::spinOnce();

		if(bumperRight || bumperLeft || bumperCenter)
			bumperIndex = 1;
		
		publishTwist(speed, 0, vel_pub);
		ros::Duration(0.1).sleep();
		distanceMoved = sqrt(pow(posX-x,2) + pow(posY-y,2));
	}
	ROS_INFO("The robot drove straight");
	publishTwist(0, 0, vel_pub);
}


void bumperRotateAngle(double angularSpeed, double angleToRotate, ros::Publisher vel_pub){
	/* This function is called only when bumper is activated */
	/* Same function as rotateAngle except all bumper detections are removed, no matter what happened, the angle rotation should be finished */
	ros::spinOnce();

	double theta = yaw;
	ROS_INFO("bumperRotateAngle: %f", angleToRotate);

	if(angleToRotate > 0){
		if(theta+angleToRotate <= pi){
			if(theta>-pi/6){
				while(yaw < theta+angleToRotate && yaw >= -pi/6){
					publishTwist(0, angularSpeed, vel_pub);
					ros::spinOnce();
				}
			}
			else{
				while(yaw < theta+angleToRotate){
					publishTwist(0, angularSpeed, vel_pub);
					ros::spinOnce();
				}
			}
		}
		else{
			while(yaw > 0 || yaw < (-pi*2+angleToRotate+theta)){
				publishTwist(0, angularSpeed, vel_pub);
				ros::spinOnce();
			}
		}
	}
	else if(angleToRotate < 0){
		if(theta+angleToRotate > -pi){
			if(theta < pi/6){
				while(yaw > theta+angleToRotate && yaw <= pi/6){
					publishTwist(0, -angularSpeed, vel_pub);
					ros::spinOnce();
				}
			}
			else{
				while(yaw > theta+angleToRotate){
					publishTwist(0, -angularSpeed, vel_pub);
					ros::spinOnce();
				}
			}
		}
		else{
			while(yaw < 0 || yaw > (pi*2+angleToRotate+theta)){
				publishTwist(0, -angularSpeed, vel_pub);
				ros::spinOnce();
			}
		}
	}
	else{
		publishTwist(0, 0, vel_pub);
		ros::spinOnce();
	}

	ROS_INFO("The robot rotated");
	publishTwist(0, 0, vel_pub);
}


void bumperDriveStraight(double speed, double distance, ros::Publisher vel_pub){
	/* This function is called only when bumper is activated */
	/* Same function as driveStraight except all bumper detections are removed, no matter what happened, the drive back should be finished */
	ros::spinOnce();

	double x = posX;
	double y = posY;
	ROS_INFO("bumperDriveStraight: %f", distance);

	double distanceMoved = sqrt(pow(posX-x,2) + pow(posY-y,2));

	while(distanceMoved < distance){
		ros::spinOnce();
		publishTwist(speed, 0, vel_pub);
		ros::Duration(0.1).sleep();
		distanceMoved = sqrt(pow(posX-x,2) + pow(posY-y,2));
	}
	ROS_INFO("The robot drove straight");
	publishTwist(0, 0, vel_pub);
}


void clearPathDriveStraight(double speed, double distance, ros::Publisher vel_pub){
	/* This function is called only when a clear path is found in the front, and the robot need to move forward a certain distance to the cross-section road as calculated */
	/* Same function as driveStraight except side angle correction is added, since this forward distance may be as long as 2m */
	ros::spinOnce();
	double x = posX;
	double y = posY;
	ROS_INFO("clearPathDriveStraight: %f", distance);

	double distanceMoved = sqrt(pow(posX-x,2) + pow(posY-y,2));
	
	bool bumperIndex = 0;

	while(distanceMoved < distance && !bumperIndex){
		ros::spinOnce();

		if ((leftDist < 0.5 && leftDist != 0) || (rightDist < 0.5 && rightDist != 0)) {     //if the robot is too close to either wall, < 0.5m
				ROS_INFO("clearPath driveStraight angleCorrection");
				if(leftDist < 0.6)
					rotateAngle(pi/6, -5.0*pi/180, vel_pub);
				else rotateAngle(pi/6, 5.0*pi/180, vel_pub);
		}

		if(bumperRight || bumperLeft || bumperCenter){
			bumperIndex = 1;
			break;
		}

		publishTwist(speed, 0, vel_pub);
		ros::Duration(0.1).sleep();
		distanceMoved = sqrt(pow(posX-x,2) + pow(posY-y,2));
	}
	ROS_INFO("The robot drove straight");
	publishTwist(0, 0, vel_pub);
}		


void randomPath(ros::Publisher vel_pub){
	/* Robot spin every 45 deg to scan the whole 360 deg, determine how many paths existing and randomly pick one path #ifndef */
	float isPath[8] = {0};   //Initialize an array storing whether it is a path in each direction
	int n_path = 0;

	ROS_INFO("Random Path");

	for (int i = 0; i < 8; i++){
		ros::spinOnce();
		ROS_INFO("midDist: %f", midDist);
		if (midDist > 1.5) {    //If the front distance is greater than 1.5, set the value in array to be 1
			isPath[i] = 1;
			n_path++;
		}
		rotateAngle(pi/6, pi/4, vel_pub);
	}

	ROS_INFO("Total paths: %d", n_path);

	// To be efficient, if the currennt position is not a dead end, eliminate the path to avoid robot return back
	if(n_path > 1){
		isPath[4] = 0;
		n_path--;
	}

	// To avoid laser reading error, if no path is detected, the robot will return back
	if(n_path == 0){
		isPath[4] = 1;
		n_path = 1;
	}

	int path_chosen = (rand() % n_path) + 1;  //Randomly choose the path
	int path_idx;   //Path index in the array

	for (int i = 0; i < 8; i++){
		if (isPath[i] == 1){
			path_chosen--;
		}
		if (path_chosen == 0){
			path_idx = i;     //Get the index of path in isPath array
			break;
		}
	}

	//ROS_INFO("path_idx: %d", path_idx);

	float turnAngle = pi/4*path_idx;
	if(turnAngle > pi)
		turnAngle = -(pi*2-turnAngle);

	rotateAngle(pi/6, turnAngle, vel_pub);
	driveStraight(0.2, 0.3, vel_pub);
}
	

int main(int argc, char **argv){
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

	ros::Subscriber odom = nh.subscribe("/odom", 1, &odomCallback);

	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	double angular = 0.0;
	double linear = 0.0;
	geometry_msgs::Twist vel;

	std::chrono::time_point<std::chrono::system_clock> start;
	start = std::chrono::system_clock::now(); /* start timer */
    uint64_t secondsElapsed = 0; // the timer just started, so we know it is less than 480, no need to check.
	
	int startInit = 10;
	ros::Rate r(10);
	while(ros::ok() && secondsElapsed <= 480)
    {
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		//ROS_INFO("Position: (%f, %f) Orientation: %f degreses Range: %f", posX, posY, yaw*180/pi, laserRange);

		ROS_INFO("leftDist, left5Dist: %f, %f", leftDist, left5Dist);
		ROS_INFO("rightDist, right5Dist: %f, %f", rightDist, right5Dist);

		//Robot Initialization to wait 10 loop cycles to avoid laser reading error
		if(startInit > 1){
			vel.angular.z = 0;
  			vel.linear.x = 0;
			vel_pub.publish(vel);
			startInit--;
		}

		//Robot starts with a randomPath, find a random heading path
		else if(startInit == 1){
			startInit--;
			randomPath(vel_pub);
		}

		//IF NO BUMPER
		else if(!bumperRight && !bumperLeft && !bumperCenter){
			//ROS_INFO("No Bumper");
			// if front distance is less than 0.8
			if (frontDist < 0.8 && frontDist != 0){
				ROS_INFO("FrontDist less than 0.8");

				// Front path is blocked, do a randomPath to randomly find a clear path
				if (abs(leftDist-rightDist) < 0.3) {
					driveStraight(0.1, 0.18, vel_pub);
					randomPath(vel_pub);
				}

				// Left Angle Correction
				else if(rightDist < leftDist){    //close to right, adjust to left
					float d = sqrt(pow(frontDist, 2)+pow(rightDist, 2)-2*frontDist*rightDist*cos(30*pi/180)); //cos law
					float angleCorrection = asin(sin(30*pi/180)*rightDist/d); //sin law to calculate angle correction
					ROS_INFO("angleCorrection: %f", angleCorrection);
					//ROS_INFO("frontDist, leftDist, rightDist: %f, %f, %f", frontDist, leftDist, rightDist);
					if(abs(angleCorrection*pi/180) > 0.05)
						rotateAngle(pi/6, angleCorrection*pi/180, vel_pub);
					else
						rotateAngle(pi/6, 0.05, vel_pub);
				}

				// Right Angle Correction
				else if(leftDist < rightDist){    //close to left, adjust to right
					float d = sqrt(pow(frontDist, 2)+pow(leftDist, 2)-2*frontDist*leftDist*cos(30*pi/180)); //cos law
					float angleCorrection = asin(sin(30*pi/180)*leftDist/d); //sin law to calculate angle correction
					//ROS_INFO("d, angleCorrection: %f, %f", d, angleCorrection);
					//ROS_INFO("frontDist, leftDist, rightDist: %f, %f, %f", frontDist, leftDist, rightDist);
					if(abs(angleCorrection*pi/180) > 0.05)
						rotateAngle(pi/6, -angleCorrection*pi/180, vel_pub);
					else
						rotateAngle(pi/6, -0.05, vel_pub);
				}

			}

			// Front Distance > 0.8 and there is a clear path detected ahead
			else if (((left5Dist - leftDist) > 0.8 && (left5Dist_last-leftDist_last) <= 0.8 && leftDist != 0 && left5Dist != 5) || ((right5Dist - rightDist) > 0.8 && (right5Dist_last-rightDist_last) <= 0.8 && rightDist != 0 && right5Dist != 5)){   //if a sudden opening on either side
				ROS_INFO("clearPath");
				// As a clear path is detected, the robot will first rotate an angle to be parallel to the current wall, then move forward a certain distance to the calculated conjuction location
				// Then rotate 90 degree to the clear path
				// If the clear path detection is a fault, the robot will rotate back
		
				if((left5Dist - leftDist) > 0.8 && (left5Dist_last-leftDist_last)<=0.8){
					ROS_INFO("Left Clearpath");
					double y = sqrt(pow(left5Dist_last,2)+pow(leftDist_last,2)-2*left5Dist_last*leftDist_last*cos(sideAngleDiff));
					double B = pi/2 - asin(leftDist_last*sin(sideAngleDiff)/y);
					double robotDirectionAngle = pi/2-B-side5tomidAngle;
					double d = left5Dist_last*sin(B);
					ROS_INFO("y, directionAngle, d: %f, %f, %f", y, robotDirectionAngle, d);
					if(d <= 2){ // If a clear path more than 2m ahead, high probability of fault detection and high risk of bumper collision, do nothing, update side_laser_last reading
						rotateAngle(pi/6, -robotDirectionAngle, vel_pub);
						clearPathDriveStraight(0.2, d+0.2, vel_pub);
						rotateAngle(pi/6, pi/2, vel_pub);
						ros::spinOnce();
						ROS_INFO("midDist: %f", midDist);
						if(midDist < 1.5) rotateAngle(pi/6, -pi/2, vel_pub);
					}
					else{
						left5Dist_last = left5Dist;
						leftDist_last = leftDist;
					}
				}
				else if((right5Dist - rightDist) > 0.8 && (right5Dist_last-rightDist_last)<=0.8){
					ROS_INFO("Right Clearpath");
					double y = sqrt(pow(right5Dist_last,2)+pow(rightDist_last,2)-2*right5Dist_last*rightDist_last*cos(sideAngleDiff));
					double B = pi/2 - asin(rightDist_last*sin(sideAngleDiff)/y);
					double robotDirectionAngle = pi/2-B-side5tomidAngle;
					double d = right5Dist_last*sin(B);
					ROS_INFO("y, directionAngle, d: %f, %f, %f", y, robotDirectionAngle, d);
					if(d <= 2){
						rotateAngle(pi/6, robotDirectionAngle, vel_pub);					
						clearPathDriveStraight(0.2, d+0.2, vel_pub);    //Drive straight
						rotateAngle(pi/6, -pi/2, vel_pub);
						ros::spinOnce();
						ROS_INFO("midDist: %f", midDist);
						if(midDist < 1.5) rotateAngle(pi/6, pi/2, vel_pub);
					}
					else{
						right5Dist_last = right5Dist;
						rightDist_last = rightDist;
					}
				}
				
			}

			//Front Distance > 0.8m but the robot is driving too close to side walls, do angle correction
			else if ((leftDist < 0.6 && leftDist != 0) || (rightDist < 0.6 && rightDist != 0)) {
				ROS_INFO("infinite loooop");
				if(abs(leftDist-rightDist) < 0.2) //if both side readings are close, which means the path is narrow, just move forward with more care
					driveStraight(0.1, 0.1, vel_pub);
				else if(leftDist < 0.6)
					rotateAngle(pi/6, -2.0*pi/180, vel_pub);
				else rotateAngle(pi/6, 2.0*pi/180, vel_pub);
			}

			//Front Distance > 0.8m and no above situation occurs, move ahead fast
			else driveStraight(0.2, 0.2, vel_pub);

		}

		//IF ANY BUMPER ACTIVATED
		else{
			bumperDriveStraight(-0.1, 0.2, vel_pub); // Drive Backwards
			if(bumperLeftIndex && !bumperRightIndex && !bumperCenterIndex)
				bumperRotateAngle(pi/6, -pi/6, vel_pub);
			else if(bumperRightIndex && !bumperLeftIndex && !bumperCenterIndex)
				bumperRotateAngle(pi/6, pi/6, vel_pub);
			else
				bumperRotateAngle(pi/6, pi/2, vel_pub);

			//Reset bumper index
			bumperCenterIndex = 0;
			bumperRightIndex = 0;
			bumperLeftIndex = 0;

			ros::spinOnce();
		}
		
		// Before going into the next loop, update the side_distance_last, prepare for clearPath calculation
		if((left5Dist-leftDist) <= 0.8){
			leftDist_last = leftDist;
			left5Dist_last = left5Dist;
		}
		if((right5Dist-rightDist) <= 0.8){
			rightDist_last = rightDist;
			right5Dist_last = right5Dist;
		}


        // The last thing to do is to update the timer.
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		r.sleep();	
	}

	return 0;
}

