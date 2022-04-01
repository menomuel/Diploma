#include "ros/ros.h"

#include <chrono>
#include <thread>
#include <iostream>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <pigpiod_if2.h>

#include <eigen3/Eigen/Dense>
#include <cmath>

// COM PORT
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

static const int pin1 = 5; //17
static const int pin2 = 13; //12
static const int pin3 = 21; //25
static const int pin4 = 27; //23

static const int pwm_freq = 400;
static const int pwm_range = 255;

static const int MIN_WIDTH = 1150;
static const int MAX_WIDTH = 1900;

static int FLOOR_WIDTH = 1150;
static int CEIL_WIDTH = 1830;

bool lockFlag = false;

double remote_u1 = 0;
double remote_u2 = 0;
double remote_u3 = 0;

geometry_msgs::PointStamped controls_msg; 

int pi = -1;

static int fd1;

//static geometry_msgs::Quaternion pwm_msg;
//ros::Publisher pwm_pub = n.advertise<geometry_msgs::Quaternion>("pwm", 0);

void pwm(int width, int sleep=0)
{
	
	if (width != 0)
	{
		if (width >= CEIL_WIDTH)
			width = CEIL_WIDTH;
		else if (width <= FLOOR_WIDTH)
			width = FLOOR_WIDTH;
	}	
	
		
	if (lockFlag)
	{
		width = 0;
	    ROS_INFO_STREAM("Abort... Motors locked\n");
	}
	
	unsigned dutycycle = static_cast<unsigned>(width/1000000. * pwm_freq * pwm_range);
	//ROS_INFO_STREAM("pwm: " << width << "\ndutycycle: " << dutycycle << "\nrange: " << pwm_range << "\nfreq: " << pwm_freq);
	
	//ROS_INFO_STREAM("pwm: " << width);
	set_PWM_dutycycle(pi, pin1, dutycycle);
	set_PWM_dutycycle(pi, pin2, dutycycle);
	set_PWM_dutycycle(pi, pin3, dutycycle);
	set_PWM_dutycycle(pi, pin4, dutycycle);
		
	if (sleep)
		std::this_thread::sleep_for(std::chrono::seconds(sleep));
	return;
}

void pwmSingle(int pin, int width, int sleep=0)
{
	if (width != 0)
	{
		if (width >= CEIL_WIDTH)
			width = CEIL_WIDTH;
		else if (width <= FLOOR_WIDTH)
			width = FLOOR_WIDTH;
	}
	
	if (lockFlag)
	{
		width = 0;
	    ROS_INFO_STREAM("Abort... Motors locked\n");
	}
	
	unsigned dutycycle = static_cast<unsigned>(width/1000000. * pwm_freq * pwm_range);
	
	//ROS_INFO_STREAM("pwm: " << width << " dutycycle: " << dutycycle << "\n");
	set_PWM_dutycycle(pi, pin, dutycycle);
	
	if (sleep)
		std::this_thread::sleep_for(std::chrono::seconds(sleep));
	return;
}

void calibrate()
{
	ROS_INFO_STREAM("Calibrating...");
	ROS_INFO_STREAM("Disconnect power and press Enter...");
	std::cin.get();
	pwm(MAX_WIDTH);
	ROS_INFO_STREAM("Connect power and press Enter to calibrate...");
	std::cin.get();
	pwm(MAX_WIDTH, 4);
	pwm(MIN_WIDTH, 8);
	ROS_INFO_STREAM("Calibrate");
	return;
}

void arm()
{
	ROS_INFO_STREAM("Arming...");
	pwm(MIN_WIDTH, 4);
	ROS_INFO_STREAM("Armed");
}

void disarm()
{
	ROS_INFO_STREAM("Disarming...");
	pwm(MIN_WIDTH);
	ROS_INFO_STREAM("Disarmed");
}


void controlsCallback(const geometry_msgs::QuaternionStamped& msg)
{
	//controls_msg = msg;

	double l = 0.12; // [m]
	double k1 = 0.1; //0.15;
	double k2 = 10;

	Eigen::Matrix4d mat;
	mat << 1./4,  k1/(4*l), -k1/(4*l),   1./(4*k2),
				 1./4,  k1/(4*l),  k1/(4*l),  -1./(4*k2),
				 1./4, -k1/(4*l),  k1/(4*l),   1./(4*k2),
				 1./4, -k1/(4*l), -k1/(4*l),  -1./(4*k2);
	
	//Eigen::Matrix4d mat;
	//mat << 1./4,  1/(4*l), -1/(4*l),   1./(4*k),
	//			 1./4,  1/(4*l),  1/(4*l),  -1./(4*k),
	//			 1./4, -1/(4*l),  1/(4*l),   1./(4*k),
	//			 1./4, -1/(4*l), -1/(4*l),  -1./(4*k);


	//INVERSE
	//mat <<    1.,  1.,  1.,  1.,
	//		   l,   l,  -l,  -l,
	//		   -l,  l,  l,   -l,
	//		   k,  -k,   k,  -k;


	Eigen::Vector4d u(msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w);

	u[0] = u[0] < remote_u1 ? u[0] : remote_u1;
	u[1] += remote_u2; // Equality!
	u[2] += remote_u3; // Equality!


	double ulim = 3; //0.7
	double unorm = sqrt(u[1]*u[1]+u[2]*u[2]);	
	if (unorm > ulim)
	{
		ROS_INFO("Norm limit reached");
		u[1] *= ulim / unorm;
		u[2] *= ulim / unorm;
	} 
	
	double u4lim = 2;
	if (u[3] > u4lim)
		u[3] = u4lim;
	else if (u[3] < -u4lim)
		u[3] = -u4lim;
	

	// ROS_INFO("Remote (%f, %f, %f):\n", remote_u1, remote_u2, remote_u3);
	//ROS_INFO_STREAM("Controls:\n" << u);    
	  
	  
	Eigen::Vector4d F = mat * u;
	//ROS_INFO_STREAM("Forces:\n" << F);

	// pwm = coef * F + bias
	double bias = 1194.1;
	double coef = 157.5;

	double pwm0 = F(0)*coef+bias;
	double pwm1 = F(1)*coef+bias;
	double pwm2 = F(2)*coef+bias;
	double pwm3 = F(3)*coef+bias;
	//ROS_INFO("pwm: %f %f %f %f", pwm0, pwm1, pwm2, pwm3);    


	pwmSingle(pin1, pwm0, 0);
	pwmSingle(pin2, pwm1, 0);
	pwmSingle(pin3, pwm2, 0);
	pwmSingle(pin4, pwm3, 0);	

	
	//geometry_msgs::QuaternionStamped pwm_msg;
	//pwm_msg.header.stamp = ros::Time::now();
	//pwm_msg.quaternion.x = pwm0;
	//pwm_msg.quaternion.y = pwm1;
	//pwm_msg.quaternion.z = pwm2;
	//pwm_msg.quaternion.w = pwm3;
	

	/*
	double PWM = 0;
	if (u[2] > 5)
	{
	PWM = 1500;
	}
	//ROS_INFO("In driver u[2] = %f\n", u[2]);
	
	//if (PWM > 1000)
	//{
	//	ROS_INFO("Hi");
	//	ssize_t stat1 = write(fd1, "A", 1);
	//	if (stat1 < 0)
	//	fprintf(stderr, "Unable to write to Serial port 1 %d\n", fd1);
	//}
	
	pwm(PWM, 0);
	*/
}

void remoteCallback(const geometry_msgs::Point& msg)
{
	remote_u1 = msg.x;
	remote_u2 = msg.y;
	remote_u3 = msg.z;
	if (msg.x == -2)
		lockFlag = true;
}


int main(int argc, char **argv)
{
	fd1 = open("/dev/ttyUSB0", O_WRONLY);
	if (fd1 == -1)
	{
		fprintf(stderr, "open_port: Unable to open /dev/ttyUSB0");
	} else
	{
		fcntl(fd1, F_SETFL, 0);
	}
	
	pi = pigpio_start(0,0);
	if (pi < 0)
	{
		fprintf(stderr, "pigpio initialisation failed\n");
		return 1;
	}
	
	set_mode(pi, pin1, PI_OUTPUT);
	set_mode(pi, pin2, PI_OUTPUT);
	set_mode(pi, pin3, PI_OUTPUT);
	set_mode(pi, pin4, PI_OUTPUT);

	set_PWM_frequency(pi, pin1, pwm_freq);
	set_PWM_frequency(pi, pin2, pwm_freq);
	set_PWM_frequency(pi, pin3, pwm_freq);
	set_PWM_frequency(pi, pin4, pwm_freq);
	
	//calibrate();
	//return 0;
  
	arm();
	//pwm(1200, 10);
	//pwm(0, 0);
	//return 0;

	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line.
	* For programmatic remappings you can use a different version of init() which takes
	* remappings directly, but for most command-line programs, passing argc and argv is
	* the easiest way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "listener");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	/**
	* The subscribe() call is how you tell ROS that you want to receive messages
	* on a given topic.  This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing.  Messages are passed to a callback function, here
	* called chatterCallback.  subscribe() returns a Subscriber object that you
	* must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	* object go out of scope, this callback will automatically be unsubscribed from
	* this topic.
	*
	* The second parameter to the subscribe() function is the size of the message
	* queue.  If messages are arriving faster than they are being processed, this
	* is the number of messages that will be buffered up before beginning to throw
	* away the oldest ones.
	*/
	ros::Subscriber subControls = n.subscribe("controls", 1, controlsCallback); 
	ros::Subscriber subRemote = n.subscribe("remote", 1, remoteCallback);
	
	ros::Publisher pwm_pub = n.advertise<geometry_msgs::QuaternionStamped>("/pwm", 1);

	/**
	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	*/
	ros::spin();
	
	/*
	while (ros::ok())
	{
		ros::spinOnce();	
		
		double l = 0.12; // [m]
		double k = 0.1; //0.15;

		Eigen::Matrix4d mat;
		mat << 1./4,  k/(4*l), -k/(4*l), -1./(4*k),
			 1./4,  k/(4*l),  k/(4*l),  1./(4*k),
			 1./4, -k/(4*l),  k/(4*l), -1./(4*k),
			 1./4, -k/(4*l), -k/(4*l),  1./(4*k);

		Eigen::Vector4d u(controls_msg.point.x, controls_msg.point.y, controls_msg.point.z, 0.);

		u[0] = u[0] < remote_u1 ? u[0] : remote_u1;
		u[1] += remote_u2; // Equality!
		u[2] += remote_u3; // Equality!

		double ulim = 3; //0.7
		double unorm = sqrt(u[1]*u[1]+u[2]*u[2]);

		
		//if (unorm > ulim)
		//{
		//u[1] *= ulim / unorm;
		//u[2] *= ulim / unorm;
		//} 
		

		// ROS_INFO("Remote (%f, %f, %f):\n", remote_u1, remote_u2, remote_u3);
		//ROS_INFO_STREAM("Controls:\n" << u);    
		  
		  
		Eigen::Vector4d F = mat * u;
		//ROS_INFO_STREAM("Forces:\n" << F);

		// pwm = coef * F + bias
		double bias = 1194.1;
		double coef = 157.5;

		double pwm0 = F(0)*coef+bias;
		double pwm1 = F(1)*coef+bias;
		double pwm2 = F(2)*coef+bias;
		double pwm3 = F(3)*coef+bias;


		//pwmSingle(pin1, pwm0, 0);
		//pwmSingle(pin2, pwm1, 0);
		//pwmSingle(pin3, pwm2, 0);
		//pwmSingle(pin4, pwm3, 0);	

		
		//geometry_msgs::QuaternionStamped pwm_msg;
		//pwm_msg.header.stamp = ros::Time::now();
		//pwm_msg.quaternion.x = pwm0;
		//pwm_msg.quaternion.y = pwm1;
		//pwm_msg.quaternion.z = pwm2;
		//pwm_msg.quaternion.w = pwm3;
		

		
		double PWM = 0;
		if (u[2] > 5)
		{
		PWM = 1500;
		}
		//ROS_INFO("In driver u[2] = %f\n", u[2]);
		
		//if (PWM > 1000)
		//{
		//	ROS_INFO("Hi");
		//	ssize_t stat1 = write(fd1, "A", 1);
		//	if (stat1 < 0)
		//	fprintf(stderr, "Unable to write to Serial port 1 %d\n", fd1);
		//}
		
		pwm(PWM, 0);

		
		geometry_msgs::QuaternionStamped pwm_msg;
		pwm_msg.header.stamp = ros::Time::now();
		pwm_msg.quaternion.x = pwm0;
		pwm_msg.quaternion.y = pwm1;
		pwm_msg.quaternion.z = pwm2;
		pwm_msg.quaternion.w = pwm3;	
		
		pwm_pub.publish(pwm_msg);
	}
	*/
	

	return 0;
}
