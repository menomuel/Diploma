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

bool wakeUp = false;

void rpyCallback(const geometry_msgs::Vector3Stamped& msg)
{
	glob_rpy_msg = msg;
    //ROS_INFO("RPY {r=%.5f, p=%.5f, y=%.5f}", msg.vector.x, msg.vector.y, msg.vector.z);
}

void angleVelCallback(const sensor_msgs::Imu& msg)
{
	glob_angleVel_msg = msg;
	if (!wakeUp)
		wakeUp = true;
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
	ros::NodeHandle n("~");

	int param_N;
	double param_rw, param_ra;
	double param_qw, param_qa;

	n.param<int>("N", param_N, 50);
	n.param<double>("rw", param_rw, 0.1);
	n.param<double>("qw", param_qw, 0.001);
	n.param<double>("ra", param_ra, 0.1);
	n.param<double>("qa", param_qa, 0.001);
	
	ROS_INFO("Steps equal to %d\n", param_N);
	ROS_INFO("velocity 'r' equal to %f\n", param_rw);
	ROS_INFO("velocity 'q' equal to %f\n", param_qw);
	ROS_INFO("angle 'r' equal to %f\n", param_ra);
	ROS_INFO("angle 'q' equal to %f\n", param_qa);
	
	
	ros::Subscriber subRPY = n.subscribe("/imu/rpy/filtered", 1, rpyCallback);
	ros::Subscriber subAngleVel = n.subscribe("/imu/data_raw", 1, angleVelCallback); //with filter subscribe on /imu/data!

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::PointStamped>("/controls", 1);
	ros::Publisher chatter_pub_mod = n.advertise<sensor_msgs::Imu>("/imu/data_mod", 1);
	ros::Publisher chatter_rpy_pred = n.advertise<geometry_msgs::Vector3Stamped>("/imu/rpy/predicted", 1);
	ros::Publisher chatter_imu_prefilter = n.advertise<sensor_msgs::Imu>("/imu/data_prefiltered", 1);
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
	int N = param_N; // 9 (experim) + 25 (acc filter)
	
	double init_val = 0;
	double acc_filter_val_x = init_val;
	double acc_filter_val_y = init_val;
	double acc_filter_val_z = init_val;
	double gyro_filter_val_x = init_val;
	double gyro_filter_val_y = init_val;
	
	double t_prev = ros::Time::now().toSec();
	double T = 0.05; // [s], filter param
	
	Eigen::Matrix2d F;
	F <<  1, 0, 
			  dt*0, 1;
	
	Eigen::Vector2d Bx {dt / I_xx, 0};
	Eigen::Vector2d By {dt / I_yy, 0};
	
	//const double q = param_q;
	Eigen::Matrix2d Q;
	Q << param_qw, 0, 
			  0, param_qa;

	//const double r = param_r;
	Eigen::Matrix2d R;
	R << param_rw, 0, 
			  0, param_ra;
	
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

	double w_x_measure, w_x_pred = init_val;
	double phi_measure, phi_pred = init_val;
	double w_y_measure, w_y_pred = init_val;
	double teta_measure, teta_pred = init_val;
	
	int count = 0;
	while (ros::ok())
	{
		ros::spinOnce();
		
		if (wakeUp)
		{
			double dt_filter = glob_angleVel_msg.header.stamp.toSec() - t_prev;
			t_prev = glob_angleVel_msg.header.stamp.toSec();
			
			acc_filter_val_x = acc_filter_val_x - 1/T * (acc_filter_val_x -  glob_angleVel_msg.linear_acceleration.x) * dt_filter;
			acc_filter_val_y = acc_filter_val_y - 1/T * (acc_filter_val_y -  glob_angleVel_msg.linear_acceleration.y) * dt_filter;
			acc_filter_val_z = acc_filter_val_z - 1/T * (acc_filter_val_z -  glob_angleVel_msg.linear_acceleration.z) * dt_filter;
			
			gyro_filter_val_x = gyro_filter_val_x - 1/T * (gyro_filter_val_x -  glob_angleVel_msg.angular_velocity.x) * dt_filter;
			gyro_filter_val_y = gyro_filter_val_y - 1/T * (gyro_filter_val_y -  glob_angleVel_msg.angular_velocity.y) * dt_filter;
			
			sensor_msgs::Imu msg_pf;
			msg_pf.header.stamp = ros::Time::now();
			msg_pf.linear_acceleration.x = acc_filter_val_x;
			msg_pf.linear_acceleration.y = acc_filter_val_y;
			msg_pf.linear_acceleration.z = acc_filter_val_z;
			msg_pf.angular_velocity.x = gyro_filter_val_x;
			msg_pf.angular_velocity.y = gyro_filter_val_y;
			
			
			//phi_measure = glob_rpy_msg.vector.x; // offset angle
			//phi_measure = std::atan2(glob_angleVel_msg.linear_acceleration.y, glob_angleVel_msg.linear_acceleration.z);
			phi_measure = std::atan2(acc_filter_val_y, acc_filter_val_z);
			
			//w_x_measure = glob_angleVel_msg.angular_velocity.x;
			w_x_measure = gyro_filter_val_x;
			
			//teta_measure = glob_rpy_msg.vector.y;
			//teta_measure = - std::atan2(glob_angleVel_msg.linear_acceleration.x, glob_angleVel_msg.linear_acceleration.z);
			teta_measure = - std::atan2(acc_filter_val_x, acc_filter_val_z);
			
			//w_y_measure = glob_angleVel_msg.angular_velocity.y;	
			w_y_measure = gyro_filter_val_y;
			
			//ROS_INFO("acc_x %f acc_y %f acc_z %f gyro_x %f gyro_y %f\n", acc_filter_val_x, acc_filter_val_y, acc_filter_val_z, gyro_filter_val_x, gyro_filter_val_y);

			// N-steps prediction forward
			
			/*
			for (int i = 0; i < N; ++i)
			{
				phi_measure += w_x_measure * dt_fwd;
				w_x_measure += (u2_kalman / I_xx) * dt_fwd;
				teta_measure += w_y_measure * dt_fwd;
				w_y_measure += (u3_kalman / I_yy) * dt_fwd;
			}
			*/
			
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
			Py = (Eigen::MatrixXd::Identity(2, 2) - Ky_k * H) * Py;
			
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
			
			/// Debug
			//u3 = glob_angleVel_msg.angular_velocity.y > 6. ? 6. : 0;
			//if (u3 > 5)
			//	ROS_INFO("msg: %f u3: %f\n", glob_angleVel_msg.angular_velocity.y, u3); 
			
			u2_kalman = u2;
			u3_kalman = u3;

			//ROS_INFO_STREAM("STEP " << count << "\n");
			//ROS_INFO_STREAM("Px: " << Px << "\n");
			//ROS_INFO_STREAM("Py: " << Py << "\n");
			//ROS_INFO("w_x_pred = %f w_y_pred = %f\n u_kalman = (%f, %f)", w_x_pred,  w_y_pred, u2_kalman, u3_kalman);

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
			
			geometry_msgs::Vector3Stamped msg_rpy_pred;
			msg_rpy_pred.header.stamp = ros::Time::now();
			msg_rpy_pred.vector.x = phi_pred;
			msg_rpy_pred.vector.y = teta_pred;
			msg_rpy_pred.vector.z = teta_measure;
		
			chatter_rpy_pred.publish(msg_rpy_pred);
			chatter_pub.publish(msg);
			chatter_pub_mod.publish(msg_mod);
			chatter_imu_prefilter.publish(msg_pf);
			
			++count;
		}
		loop_rate.sleep();
	}


	return 0;
}
