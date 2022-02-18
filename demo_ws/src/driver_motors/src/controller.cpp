#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>


#include <sstream>

#include <eigen3/Eigen/Dense>

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
/*
void timerCallback(const ros::TimerEvent& event)
{
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
	
	double u1 = m * g;
	double u2 = I_xx * (-(a_phi+k_phi)*phiVel - a_phi*k_phi*phi);
	double u3 = I_yy * (-(a_teta+k_teta)*tetaVel - a_teta*k_teta*teta);
	
	//ROS_INFO("u={%.1f, %.1f, %.1f}", u1, u2, u3);
	//ROS_INFO("currVel={%f, %f}, Vel={%f, %f}, u={%.1f, %.1f}", currPhiVel, currTetaVel, phiVel, tetaVel, u2, u3);
	
	//geometry_msgs::Point msg;
    geometry_msgs::PointStamped msg;
    
    msg.header.stamp = ros::Time::now();
    msg.point.x = u1;
    msg.point.y = u2;
    msg.point.z = u3;
    
    chatter_pub.publish(msg);
}
*/

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "controller");
	ros::NodeHandle n;


	ros::Subscriber subRPY = n.subscribe("/imu/rpy/filtered", 1, rpyCallback);
	ros::Subscriber subAngleVel = n.subscribe("/imu/data", 1, angleVelCallback);

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::PointStamped>("/controls", 1);
	ros::Publisher chatter_pub_mod = n.advertise<sensor_msgs::Imu>("/imu/data_mod", 1);
	//ros::Publisher chatter_pub = n.advertise<geometry_msgs::Vector3Stamped>("controls", 1);
	ros::Rate loop_rate(200);

	//ros::Timer timer = n.createTimer(ros::Duration(0.005), timerCallback);
	//ros::spin();


	// From article
	double m = 0.7; // [kg]
	double g = 9.8; // [m/c^2]

	double I_xx = 0.0464; // [kg*m^2]
	double I_yy = 0.0464; // [kg*m^2]

	double k_teta, a_teta, k_phi, a_phi;
	k_teta = a_teta = k_phi = a_phi = 16.0;

	double phiVel = 0;
	double tetaVel = 0;
	
	double dt = 1./200;
	double dt_fwd = 1. / 500;
	int N = 9;	
	
	Eigen::Matrix2d F;
	F <<  1, 0, 
			  dt, 1;
	
	Eigen::Vector2d Bx {dt / I_xx, 0};
	Eigen::Vector2d By {dt / I_yy, 0};
	
	const double q = 0.001;
	Eigen::Matrix2d Q;
	Q << q, 0, 
			  0, q;

	const double r = 0.1;
	Eigen::Matrix2d R;
	R << r, 0, 
			  0, r;
	
	const double p = 0.1; 
	Eigen::Matrix2d Px;
	Px << p, 0, 
			  0, p;
	Eigen::Matrix2d Py;
	Py << p, 0, 
			  0, p;
			  
	Eigen::Matrix2d H = Eigen::MatrixXd::Identity(2, 2);
	/*
	std::cout << "F: " << F << std::endl;
	std::cout << "Bx: " << Bx << std::endl;
	std::cout << "R: " << R << std::endl;
	std::cout << "Q: " << Q << std::endl;
	std::cout << "Px: " << Px << std::endl;
	*/
	double u2_kalman = 0;
	double u3_kalman = 0;

	double w_x_measure, w_x_pred = 0;
	double phi_measure, phi_pred = 0;
	double w_y_measure, w_y_pred = 0;
	double teta_measure, teta_pred = 0;

	while (ros::ok())
	{
		ros::spinOnce();

		phi_measure = glob_rpy_msg.vector.x + 0.03; // offset angle
		w_x_measure = glob_angleVel_msg.angular_velocity.x;
		teta_measure = glob_rpy_msg.vector.y;
		w_y_measure = glob_angleVel_msg.angular_velocity.y;

		// N-steps prediction forward
		for (int i = 0; i < N; ++i)
		{
			phi_measure += w_x_measure * dt_fwd;
			w_x_measure += (u2_kalman / I_xx) * dt_fwd;
			teta_measure += w_y_measure * dt_fwd;
			w_y_measure += (u3_kalman / I_yy) * dt_fwd;
		}
		
		Eigen::Vector2d Xx_k_prev {w_x_pred, phi_pred};
		Eigen::Vector2d Xx_k = F * Xx_k_prev + Bx * u2_kalman;
		Eigen::Vector2d Xy_k_prev {w_y_pred, teta_pred};
		Eigen::Vector2d Xy_k = F * Xy_k_prev + By * u3_kalman;
		
		Px = F * Px * F.transpose() + Q;
		Py = F * Py * F.transpose() + Q;

		Eigen::Vector2d Zx_k {w_x_measure, phi_measure};
		Eigen::Vector2d Yx_k = Zx_k - H * Xx_k;
		Eigen::Vector2d Zy_k {w_y_measure, teta_measure};
		Eigen::Vector2d Yy_k = Zy_k - H * Xy_k;
		
		Eigen::Matrix2d Sx_k = H * Px * H.transpose() + R;
		Eigen::Matrix2d Kx_k = Px * H.transpose() * Sx_k.inverse();
		Eigen::Matrix2d Sy_k = H * Py * H.transpose() + R;
		Eigen::Matrix2d Ky_k = Py * H.transpose() * Sy_k.inverse();
		
		Xx_k += Kx_k * Yx_k;
		Px = (Eigen::MatrixXd::Identity(2, 2) - Kx_k * H) * Px;
		Xy_k += Ky_k * Yy_k;
		Py *= (Eigen::MatrixXd::Identity(2, 2) - Ky_k * H);
		
		//double k_y = P_y_plus / (P_y_plus + R);
		//w_y += k_y * Y_y;
		//P_y *= (1 - k_y);
		
		w_x_pred = Xx_k(0);
		phi_pred = Xx_k(1);
		w_y_pred = Xy_k(0);
		teta_pred = Xy_k(1);
		
		
		double u1 = m * g;
		double u2 = I_xx * (-(a_phi+k_phi)*w_x_pred - a_phi*k_phi*phi_pred);
		double u3 = I_yy * (-(a_teta+k_teta)*w_y_pred - a_teta*k_teta*teta_pred);
		
		u2_kalman = u2;
		u3_kalman = u3;

		//ROS_INFO("u={%.1f, %.1f, %.1f}", u1, u2, u3);
		//ROS_INFO("currVel={%f, %f}, Vel={%f, %f}, u={%.1f, %.1f}", currPhiVel, currTetaVel, phiVel, tetaVel, u2, u3);
	
		geometry_msgs::PointStamped msg;
		msg.header.stamp = ros::Time::now();
		msg.point.x = u1;
		msg.point.y = u2;
		msg.point.z = u3;
			
		
		sensor_msgs::Imu msg_mod = glob_angleVel_msg;
		msg_mod.angular_velocity.x = w_x_pred;
		msg_mod.angular_velocity.y = w_y_pred;
	
		chatter_pub.publish(msg);
		chatter_pub_mod.publish(msg_mod);
		

		loop_rate.sleep();
	}


	return 0;
}
