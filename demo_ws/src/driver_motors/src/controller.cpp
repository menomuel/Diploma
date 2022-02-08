#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>


#include <sstream>

geometry_msgs::Vector3Stamped glob_rpy_msg; 
sensor_msgs::Imu glob_angleVel_msg;

void rpyCallback(const geometry_msgs::Vector3Stamped& msg)
{
	glob_rpy_msg = msg;
    //ROS_INFO("RPY {r=%.5f, p=%.5f, y=%.5f}", msg.vector.x, msg.vector.y, msg.vector.z);
}

void angleVelCallback(const sensor_msgs::Imu& msg)
{
	glob_angleVel_msg = msg;
	//ROS_INFO("Receive angular velocity {%.1f, %.1f, %.1f}", msg.angular_velocity.x,
	//														msg.angular_velocity.y,
	//														msg.angular_velocity.z);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
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
  ros::init(argc, argv, "controller");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  
  
  ros::Subscriber subRPY = n.subscribe("/imu/rpy/filtered", 1, rpyCallback);
  ros::Subscriber subAngleVel = n.subscribe("/imu/data", 1, angleVelCallback);

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Point>("controls", 1);


  ros::Rate loop_rate(200);
  
  // From article
  double m = 0.7; // [kg]
  double g = 9.8; // [m/c^2]
  
  double I_xx = 0.0464; // [kg*m^2]
  double I_yy = 0.0464; // [kg*m^2]
  
  double k_teta, a_teta, k_phi, a_phi;
  k_teta = a_teta = k_phi = a_phi = 16.0;
	
  double phiVel = 0;
  double tetaVel = 0;	
  double k = 50;
  double dt = 0.01;
	
  
  while (ros::ok())
  {
	ros::spinOnce();
	
	double phi = glob_rpy_msg.vector.x;
	double teta = glob_rpy_msg.vector.y;
	
	//phi = 0;
	//teta = 0;
	
	double currPhiVel = glob_angleVel_msg.angular_velocity.x;
	double currTetaVel = glob_angleVel_msg.angular_velocity.y;
	
	//phiVel -= k * (phiVel - currPhiVel) * dt;
	//tetaVel -= k * (tetaVel - currTetaVel) * dt;
	
	phiVel = currPhiVel / 1;
	tetaVel = currTetaVel / 1;
	
	
	double u1 = 0;
	double u2 = currPhiVel;
	double u3 = currTetaVel;
	
	//double u1 = m * g;
	//double u2 = I_xx * (-(a_phi+k_phi)*phiVel - a_phi*k_phi*phi);
	//double u3 = I_yy * (-(a_teta+k_teta)*tetaVel - a_teta*k_teta*teta);
	
	//ROS_INFO("u={%.1f, %.1f, %.1f}", u1, u2, u3);
	//ROS_INFO("currVel={%f, %f}, Vel={%f, %f}, u={%.1f, %.1f}", currPhiVel, currTetaVel, phiVel, tetaVel, u2, u3);
	
	geometry_msgs::Point msg;
    msg.x = u1;
    msg.y = u2;
    msg.z = u3;
    
    chatter_pub.publish(msg);
    loop_rate.sleep();
  }
  
/*
  ros::Rate loop_rate(1);
  
  int count = 900;
  while (ros::ok())
  {
    geometry_msgs::Point msg;
    msg.x = count;
    msg.y = count;
    msg.z = count;

    ROS_INFO("{%.1f, %.1f, %.1f}", msg.x, msg.y, msg.z);
    
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    if (count == 0)
	break;
    count += 50;
    if (count > 1830)
	count = 0;
  }
*/	

  return 0;
}
