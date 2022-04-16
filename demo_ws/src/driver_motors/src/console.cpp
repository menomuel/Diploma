#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>

//#include <iostream>
#include <ncurses.h>

// torque keys
static constexpr char TORQ_INC = 0x6F; // 'o'
static constexpr char TORQ_DEC = 0x6C; // 'l'
static constexpr char TORQ_OFF = 0x20; // ' '


// roll keys
static constexpr char ROLL_INC = 0x64; // 'd'
static constexpr char ROLL_DEC = 0x61; // 'a'

// pitch keys
static constexpr char PITCH_INC = 0x77; // 'w'
static constexpr char PITCH_DEC = 0x73; // 's'

static geometry_msgs::PointStamped msg;

int main(int argc, char **argv)
{
  initscr();
  halfdelay(1);  

  msg.point.x = 0; msg.point.y = 0; msg.point.z = 0;
	
  ros::init(argc, argv, "console");

  ros::NodeHandle n;
  ros::Publisher remote_pub = n.advertise<geometry_msgs::PointStamped>("/remote", 1);
  
  ros::Rate loop_rate(100);
  
  while (ros::ok())
  {
	char c = getch();
	double step = 0.25;
	double stepRoll = 0.2;
	double stepPitch = 0.2;
	switch (c)
	{
		case TORQ_INC:
			msg.point.x += step;
			break;

		case TORQ_DEC:
			msg.point.x -= step;
			break;
			
		case TORQ_OFF:
			msg.point.x = -2;
			break;
		
		case ROLL_INC:
			msg.point.y = stepRoll;
			break;
		
		case ROLL_DEC:
			msg.point.y = -stepRoll;
			break;

		case PITCH_INC:
			msg.point.z = stepPitch;
			break;

		case PITCH_DEC:
			msg.point.z = -stepPitch;
			break;

		default:
			msg.point.y = 0;
			msg.point.z = 0;
			break;	
	}

    //printw("Console controls: {%.1f, %.1f, %.1f}\n", msg.x, msg.y, msg.z);
    
    msg.header.stamp = ros::Time::now();
    remote_pub.publish(msg);

    ros::spinOnce();
  }
  
  endwin();  
  return 0;
}
