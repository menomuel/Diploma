#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#include <vector>
#include <queue>
#include <deque>
#include <sstream>

#include <eigen3/Eigen/Dense>

#include <iostream>

geometry_msgs::Vector3Stamped glob_rpy_msg; 
sensor_msgs::Imu glob_angleVel_msg;
geometry_msgs::PoseStamped glob_pose_msg; 

bool wakeUp = false;
bool cameraUp = false;

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

void poseCallback(const geometry_msgs::PoseStamped& msg)
{
	glob_pose_msg = msg;
	
	double x = msg.pose.position.x;
	double y = msg.pose.position.y;
	double z = msg.pose.position.z;
	
	if (!cameraUp)
		cameraUp = true;
	
	//ROS_INFO("Pose x=%.4f y=%.4f z=%.4f}", x, y, z);
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
	
	ROS_INFO("velocity 'r' equal to %f\n", param_rw);
	ROS_INFO("velocity 'q' equal to %f\n", param_qw);
	ROS_INFO("angle 'r' equal to %f\n", param_ra);
	ROS_INFO("angle 'q' equal to %f\n", param_qa);
	
	
	ros::Subscriber subRPY = n.subscribe("/imu/rpy/filtered", 1, rpyCallback);
	ros::Subscriber subAngleVel = n.subscribe("/imu/data_raw", 1, angleVelCallback); //with filter subscribe on /imu/data!
	ros::Subscriber subPose = n.subscribe("/mavros/vision_pose/pose", 1, poseCallback);

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::PointStamped>("/controls", 1);
	ros::Publisher chatter_imu_prefilter = n.advertise<sensor_msgs::Imu>("/imu/data_prefiltered", 1);
	
	ros::Publisher chatter_imu_stepped = n.advertise<sensor_msgs::Imu>("/imu/data_stepped", 1);
	ros::Publisher chatter_rpy_stepped = n.advertise<geometry_msgs::Vector3Stamped>("/imu/rpy/stepped", 1);
	
	ros::Publisher chatter_pub_mod = n.advertise<sensor_msgs::Imu>("/imu/data_mod", 1);
	ros::Publisher chatter_rpy_pred = n.advertise<geometry_msgs::Vector3Stamped>("/imu/rpy/predicted", 1);
	
	
	ros::Rate loop_rate(200);

	//ros::Timer timer = n.createTimer(ros::Duration(0.005), timerCallback);
	//ros::spin();


	// From article
	double m = 0.5; // [kg]
	double g = 9.8; // [m/c^2]

	double I_xx = 0.01; // 0.0464; // [kg*m^2]
	double I_yy = 0.01; // 0.0464; // [kg*m^2]

	double k_x, a_x, k_y, a_y, k_z, a_z;
	k_x = a_x = k_y = a_y = k_z = a_z = 4.0;

	double k_teta, a_teta, k_phi, a_phi;
	k_teta = a_teta = k_phi = a_phi = 16.0;

	double phiVel = 0;
	double tetaVel = 0;
	
	double dt = 1./200;
	double dt_fwd = 1. / 200; 
	//int N = param_N;
	constexpr int N = 10;
	ROS_INFO("Steps equal to %d\n", N);
	
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
			  dt, 1;
	
	Eigen::Vector2d Bx {dt / I_xx, 0}; //SHOULD BE PLUS
	Eigen::Vector2d By {dt / I_yy, 0}; //SHOULD BE PLUS
	
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


	std::deque<double> w_x_pred_q(N, 0);
	std::deque<double> w_y_pred_q(N, 0);
	
	std::deque<double> u2_kalman_q(N, 0);
	std::deque<double> u3_kalman_q(N, 0);

	double w_x_measure, w_x_pred = init_val;
	double phi_measure, phi_pred = init_val;
	double w_y_measure, w_y_pred = init_val;
	double teta_measure, teta_pred = init_val;
	
	double x_cam_prev = 0, y_cam_prev = 0, z_cam_prev = 0;
	
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
			double phi_offset = -0.071;
			phi_measure = std::atan2(acc_filter_val_y, acc_filter_val_z) - phi_offset;
			
			//w_x_measure = glob_angleVel_msg.angular_velocity.x;
			w_x_measure = gyro_filter_val_x;
			
			//teta_measure = glob_rpy_msg.vector.y;
			//teta_measure = - std::atan2(glob_angleVel_msg.linear_acceleration.x, glob_angleVel_msg.linear_acceleration.z);
			double teta_offset = -0.018;
			teta_measure = - std::atan2(acc_filter_val_x, acc_filter_val_z) - teta_offset;
			
			//w_y_measure = glob_angleVel_msg.angular_velocity.y;	
			w_y_measure = gyro_filter_val_y;
			
			//ROS_INFO("acc_x %f acc_y %f acc_z %f gyro_x %f gyro_y %f\n", acc_filter_val_x, acc_filter_val_y, acc_filter_val_z, gyro_filter_val_x, gyro_filter_val_y);
			
			

			double phi_stepped = phi_measure;
			double w_x_stepped = w_x_measure;
			
			double teta_stepped = teta_measure;
			double w_y_stepped = w_y_measure;
			
			for (int i = 0; i < N; ++i)
			{
				w_x_stepped += (u2_kalman_q[i] / I_xx) * dt_fwd;
				phi_stepped += w_x_pred_q[i] * dt_fwd;
				//phi_stepped += w_x_stepped * dt_fwd;
				w_y_stepped  += (u3_kalman_q[i] / I_yy) * dt_fwd;
				teta_stepped += w_y_pred_q[i] * dt_fwd;
				//teta_stepped += w_y_stepped * dt_fwd;
			}
			
			
			sensor_msgs::Imu msg_st;
			msg_st.header.stamp = ros::Time::now();
			msg_st.angular_velocity.x = w_x_stepped;
			msg_st.angular_velocity.y = w_y_stepped;
			
			geometry_msgs::Vector3Stamped msg_rpy_stepped;
			msg_rpy_stepped.header.stamp = ros::Time::now();
			msg_rpy_stepped.vector.x = phi_stepped;
			msg_rpy_stepped.vector.y = teta_stepped;			
			
			Eigen::Vector2d Xx_k_prev {w_x_pred, phi_pred};
			Eigen::Vector2d Xx_k = F * Xx_k_prev + Bx * u2_kalman;
			Eigen::Vector2d Xy_k_prev {w_y_pred, teta_pred};
			Eigen::Vector2d Xy_k = F * Xy_k_prev + By * u3_kalman;
			
			Px = F * Px * F.transpose() + Q;
			Py = F * Py * F.transpose() + Q;
			
			
			Eigen::Vector2d Zx_k {w_x_stepped, phi_stepped};
			Eigen::Vector2d Yx_k = Zx_k - H * Xx_k;
			Eigen::Vector2d Zy_k {w_y_stepped, teta_stepped};
			Eigen::Vector2d Yy_k = Zy_k - H * Xy_k;
			
			Eigen::Matrix2d Sx_k = H * Px * H.transpose() + R;
			Eigen::Matrix2d Kx_k = Px * H.transpose() * Sx_k.inverse();
			Eigen::Matrix2d Sy_k = H * Py * H.transpose() + R;
			Eigen::Matrix2d Ky_k = Py * H.transpose() * Sy_k.inverse();
			
			Xx_k += Kx_k * Yx_k;
			Px = (Eigen::MatrixXd::Identity(2, 2) - Kx_k * H) * Px;
			Xy_k += Ky_k * Yy_k;
			Py = (Eigen::MatrixXd::Identity(2, 2) - Ky_k * H) * Py;
			
			w_x_pred = Xx_k(0);
			phi_pred = Xx_k(1);
			w_y_pred = Xy_k(0);
			teta_pred = Xy_k(1);
			
			
			double x_ref = 0, y_ref = 0, z_ref = 1;
			double x_cam = 0, y_cam = 0, z_cam = 0;
			if (cameraUp)
			{
				x_cam = glob_pose_msg.pose.position.x;
				y_cam = glob_pose_msg.pose.position.y;
				z_cam = glob_pose_msg.pose.position.z;
			}
			double dx_cam = 0; //x_cam - x_cam_prev;
			double dy_cam = 0; //y_cam - y_cam_prev;
			double dz_cam = 0; //z_cam - z_cam_prev;
			x_cam_prev = x_cam;
			y_cam_prev = y_cam;
			z_cam_prev = z_cam;
			
			double H_xx = m * (-(a_x+k_x)*dx_cam - a_x*k_x*(x_cam-x_ref));
			double H_yy = m * (-(a_y+k_y)*dy_cam - a_y*k_y*(y_cam-y_ref));
			double H_zz = m * (-(a_z+k_z)*dz_cam - a_z*k_z*(z_cam-z_ref)) + m * g;
			
			double teta_ref = std::atan2(H_xx, H_zz);
			//double teta_ref = 0;
			double phi_ref = std::atan2(-H_yy, sqrt(H_xx*H_xx + H_zz*H_zz));
			//double phi_ref = 0;
			
			double u1 = sqrt(H_xx*H_xx + H_yy*H_yy + H_zz*H_zz);
			if (u1 >= 8)
				u1 = 8;
			//double u1 = m * g;
			double u2 = I_xx * (-(a_phi+k_phi)*w_x_pred - a_phi*k_phi*(phi_pred-phi_ref));
			double u3 = I_yy * (-(a_teta+k_teta)*w_y_pred - a_teta*k_teta*(teta_pred-teta_ref));
			/// TEST CONTROLLER
			//double u2 = I_xx * (-(a_phi+k_phi)*w_x_measure - a_phi*k_phi*phi_measure);
			//double u3 = I_yy * (-(a_teta+k_teta)*w_y_measure - a_teta*k_teta*teta_measure);
			
			/// Debug
			//u3 = glob_angleVel_msg.angular_velocity.y > 6. ? 6. : 0;
			//if (u3 > 5)
			//	ROS_INFO("msg: %f u3: %f\n", glob_angleVel_msg.angular_velocity.y, u3); 
			
			u2_kalman = u2;
			u3_kalman = u3;
			
			u2_kalman_q.pop_front();
			u3_kalman_q.pop_front();
			u2_kalman_q.push_back(u2);
			u3_kalman_q.push_back(u3);

			w_x_pred_q.pop_front();
			w_y_pred_q.pop_front();
			w_x_pred_q.push_back(w_x_pred);
			w_y_pred_q.push_back(w_y_pred);

			//for (int i=0; i<N; ++i)
			//	ROS_INFO("u2[%d]=%f ", i, u2_kalman_q[i]);
			//ROS_INFO("u2_kalman %f\n", u2_kalman);
			
			//for (int i=0; i<N; ++i)
			//	ROS_INFO("u3[%d]=%f ", i, u3_kalman_q[i]);
			//ROS_INFO("u3_kalman %f\n", u3_kalman);
			
			//for (int i=0; i<N; ++i)
			//	ROS_INFO("w_x[%d]=%f ", i, w_x_pred_q[i]);
			//ROS_INFO("\n");

			//ROS_INFO_STREAM("STEP " << count << "\n");
			//ROS_INFO_STREAM("Px: " << Px << "\n");
			//ROS_INFO_STREAM("Py: " << Py << "\n");
			//ROS_INFO("u={%.1f, %.1f}\n", u2, u3);
			//ROS_INFO("w_x_measure=%f, w_x_stepped=%f, w_x_pred=%f, u2=%f", w_x_measure, w_x_stepped, w_x_pred,  u2);
			//ROS_INFO("w_y_measure=%f, w_y_stepped=%f, w_y_pred=%f, u3=%f", w_y_measure, w_y_stepped, w_y_pred,  u3);
			
			//ROS_INFO("phi_measure=%f, phi_stepped=%f, phi_pred=%f",  phi_measure, phi_stepped, phi_pred);
			//ROS_INFO("teta_measure=%f, teta_stepped=%f, teta_pred=%f",  teta_measure, teta_stepped, teta_pred);
		
			//ROS_INFO("w_x_measure=%f, w_x_stepped=%f, w_x_pred=%f,  phi_measure=%f, phi_stepped=%f, phi_pred=%f, u2=%f",  w_x_measure, w_x_stepped, w_x_pred, phi_measure, phi_stepped, phi_pred, u2);
		
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
		
		
			chatter_pub.publish(msg);
			
			chatter_imu_prefilter.publish(msg_pf);
			chatter_imu_stepped.publish(msg_st);
			chatter_rpy_stepped.publish(msg_rpy_stepped);
			
			chatter_pub_mod.publish(msg_mod);
			chatter_rpy_pred.publish(msg_rpy_pred);
			
			++count;
		}
		loop_rate.sleep();
	}


	return 0;
}
