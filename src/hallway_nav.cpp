#include <iostream>
#include <stdio.h>
#include <string.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "roboteq/MotorCommands.h"

//const int ARRAY_SIZE = 726;

float angleMin;
float angleMax;
float angleInc;
float timeInc;
float scanTime;
float rangeMin;
float rangeMax;
float rangeData[726];
int counter = 0;
ros::Publisher publisher;

void autonomousHallwayNav(float range[])
{
	int leftMotor = 100;
	int rightMotor = 100;
	bool leftSideDetect = false;
	bool rightSideDetect = false;
	bool frontSideDetect = false;

	ROS_INFO("Checking Sides: ");

	//right side
	for(int i = 70; i < 150; i++)
	{
		if(range[i] < .5 && range[i] > 0.02)
		{
			ROS_INFO("%i %f", i, range[i]);
			ROS_INFO("Right side is too close!");
			rightSideDetect = true;
			break;
		}
	}

	//front 
	for(int i = 353; i < 373; i++)
	{
		if(range[i] < .5 && range[i] > 0.02)
		{
			ROS_INFO("Obstacle in front, stopping!");
			frontSideDetect = true;
			break;
		}
	}

	//left
	for(int i = 576; i < 656; i++)
	{
		if(range[i] < .5 && range[i] > 0.02)
		{
			ROS_INFO("Left side is too close!");
			leftSideDetect = true;
			break;
		}
	}

	//left side
	if(leftSideDetect && !rightSideDetect && !frontSideDetect)
	{
		leftMotor = 100;
		rightMotor = 75;
	}

	//right side
	else if(!leftSideDetect && rightSideDetect && !frontSideDetect)
	{
		leftMotor = 75;
		rightMotor = 100;
	}

	//front and left
	else if(leftSideDetect && !rightSideDetect && frontSideDetect)
	{
		leftMotor = 75;
		rightMotor = -25;
	}

	//front and right
	else if(!leftSideDetect && rightSideDetect && frontSideDetect)
	{
		leftMotor = -25;
		rightMotor = 75;
	}

	//front and both sides
	else if(leftSideDetect && rightSideDetect && frontSideDetect)
	{
		rightMotor = 0;
		leftMotor = 0;
		ROS_INFO("Help STUCK!!!");
	}

	//both sides and not front
	else if(leftSideDetect && rightSideDetect && !frontSideDetect)
	{
		rightMotor = 100;
		leftMotor = 100;
	}

	//only front detect
	else if(frontSideDetect)
	{
		rightMotor = -75;
		leftMotor = 75;
	}

	//nothing detected
	else
	{
		ROS_INFO("Centering...");

		float leftDistance = 0;
		float rightDistance = 0;

		//left side
		for(int i = 586; i < 676; i++)
		{
			if(range[i] < 0.02)
			{
				leftDistance += 5;
			}
			else
			{
				leftDistance += range[i];
			}
		}

		leftDistance /= 90;

		//right side
		for(int i = 50; i < 140; i++)
		{
			if(range[i] < 0.02)
			{
				rightDistance += 5;
			}
			else
			{
				rightDistance += range[i];
			}
		}

		rightDistance /= 90;

		if(rightDistance < leftDistance && leftDistance != 0)
		{
			ROS_INFO("Centering to right...");
			leftMotor = ((rightDistance/leftDistance) + 0.3) * 100.0;
			rightMotor = 100;
		}
		else if(leftDistance < rightDistance && rightDistance != 0)
		{
			ROS_INFO("Centering to left...");
			leftMotor = 100;
			rightMotor = ((leftDistance/rightDistance) + 0.3) * 100.0;
		}
		else
		{
			leftMotor = 100;
			rightMotor = 100;
		}
	}

	// Limits
	if(leftMotor > 100)
	{
		leftMotor = 100;
	}

	if(rightMotor > 100)
	{
		rightMotor = 100;
	}

	//Send motor commands to motorController...
	roboteq::MotorCommands msg;
	msg.leftMotor = rightMotor * .5;
	msg.rightMotor = leftMotor * .5;

	ROS_INFO("Sending motor msg %i %i", msg.leftMotor, msg.rightMotor);
	publisher.publish(msg);
}

void processRangefinderData(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int array_size = msg->ranges.size();
	//ROS_INFO("Header: %i", msg->header);
	angleMin = (float)(msg->angle_min);
	angleMax = (float)(msg->angle_max);
	angleInc = (float)(msg->angle_increment);
	timeInc = (float)(msg->time_increment);
	scanTime = (float)(msg->scan_time);
	rangeMin = (float)(msg->range_min);
	rangeMax = (float)(msg->range_max);

	for(int i = 0; i < array_size; i++)
	{
		if((float)msg->ranges[i] == std::numeric_limits<float>::quiet_NaN())
		{
			ROS_INFO("SHIT THINGS ARE BLOWING UP");
			rangeData[i] = rangeMax;
		}
		else
		{
			rangeData[i] = (float)(msg->ranges[i]);
		}
	}


	if(counter < 20)
	{
		/*ROS_INFO("angleMin = %f", angleMin);
		ROS_INFO("angleMax = %f", angleMax);
		ROS_INFO("angleInc = %f", angleInc);
		ROS_INFO("timeInc = %f", timeInc);
		ROS_INFO("scanTime = %f", scanTime);
		ROS_INFO("rangeMin = %f", rangeMin);
		ROS_INFO("rangeMax = %f", rangeMax);*/

		//ROS_INFO("random point %f", rangeData[363]);

		/*for(int i = 0; i < array_size; i++)
		{
			ROS_INFO("scanData = %f", rangeData[i]);
		}*/

		counter++;
	}

	ROS_INFO("random point %f", rangeData[363]);

	/*ROS_INFO("angleMin = %f", angleMin);
	ROS_INFO("angleMax = %f", angleMax);
	ROS_INFO("rangeMin = %f", rangeMin);
	ROS_INFO("rangeMax = %f", rangeMax);*/

	autonomousHallwayNav(rangeData);
}

int main(int argc, char **argv)
{
	// Start up ROS
	//string rfPort = "/dev/ttyACM0";

	ROS_INFO("Starting hallway_nav node %s", "/dev/ttyACM0");

	ros::init(argc, argv, "hallway_nav");
	ros::NodeHandle node;
	
	// Subscribe to scan topic 
	//string topicName = "scan";

	ROS_INFO("Subscribing to topic: %s", "scan");
	ros::Subscriber sub = node.subscribe("scan", 1, processRangefinderData);

	publisher = node.advertise<roboteq::MotorCommands>("motorSpeeds", 1);


	ROS_INFO("Press Ctrl-C to kill node.");

	ros::spin();

	ros::shutdown();

	return 0;
}