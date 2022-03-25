#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>

#include <vector>
#include <deque>

#include <eigen3/Eigen/Dense>

#include <iostream>

geometry_msgs::Vector3Stamped glob_rpy_msg; 
sensor_msgs::Imu glob_angleVel_msg;
geometry_msgs::PoseStamped glob_pose_msg; 

bool imuUp = false;
bool cameraUp = false;

geometry_msgs::Vector3 QuatToEuler(const geometry_msgs::Quaternion& msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    
    //ROS_INFO("Current camera RPY %f %f %f", roll, pitch, yaw);

    return rpy;
}


void rpyCallback(const geometry_msgs::Vector3Stamped& msg)
{
	glob_rpy_msg = msg;
    //ROS_INFO("RPY {r=%.5f, p=%.5f, y=%.5f}", msg.vector.x, msg.vector.y, msg.vector.z);
}

void angleVelCallback(const sensor_msgs::Imu& msg)
{
	glob_angleVel_msg = msg;
	if (!imuUp)
		imuUp = true;
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

	int param_N, _param_N;
	double param_rw, param_ra;
	double param_qw, param_qa;

	n.param<int>("N", param_N, 10);
	n.param<int>("newN", _param_N, 4);
	n.param<double>("rw", param_rw, 0.1);
	n.param<double>("qw", param_qw, 0.001);
	n.param<double>("ra", param_ra, 0.1);
	n.param<double>("qa", param_qa, 0.001);
	
	ROS_INFO("velocity 'r' equal to %f", param_rw);
	ROS_INFO("velocity 'q' equal to %f", param_qw);
	ROS_INFO("angle 'r' equal to %f", param_ra);
	ROS_INFO("angle 'q' equal to %f", param_qa);
	
	
	ros::Subscriber subRPY = n.subscribe("/imu/rpy/filtered", 1, rpyCallback);
	ros::Subscriber subAngleVel = n.subscribe("/imu/data_raw", 1, angleVelCallback); //with filter subscribe on /imu/data!
	ros::Subscriber subPose = n.subscribe("/mavros/vision_pose/pose", 1, poseCallback);

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::PointStamped>("/controls", 1);
	ros::Publisher chatter_imu_prefilter = n.advertise<sensor_msgs::Imu>("/imu/data_prefiltered", 1);
	
	ros::Publisher chatter_imu_stepped = n.advertise<sensor_msgs::Imu>("/imu/data_stepped", 1);
	ros::Publisher _chatter_imu_stepped = n.advertise<sensor_msgs::Imu>("/imu/_data_stepped", 1);
	ros::Publisher chatter_rpy_stepped = n.advertise<geometry_msgs::Vector3Stamped>("/imu/rpy/stepped", 1);
	ros::Publisher _chatter_rpy_stepped = n.advertise<geometry_msgs::Vector3Stamped>("/imu/rpy/_stepped", 1);
	
	ros::Publisher chatter_pub_mod = n.advertise<sensor_msgs::Imu>("/imu/data_mod", 1);
	ros::Publisher _chatter_pub_mod = n.advertise<sensor_msgs::Imu>("/imu/_data_mod", 1);
	ros::Publisher chatter_rpy_pred = n.advertise<geometry_msgs::Vector3Stamped>("/imu/rpy/predicted", 1);
	ros::Publisher _chatter_rpy_pred = n.advertise<geometry_msgs::Vector3Stamped>("/imu/rpy/_predicted", 1);
	
	ros::Publisher pub_cam_rpy = n.advertise<geometry_msgs::Vector3>("/camera/rpy", 1);
	
	ros::Rate loop_rate(200);

	//ros::Timer timer = n.createTimer(ros::Duration(0.005), timerCallback);
	//ros::spin();


	// From article
	double m = 0.5; // [kg]
	double g = 9.8; // [m/c^2]

	double I_xx = 0.01; // 0.0464; // [kg*m^2]
	double I_yy = 0.01; // 0.0464; // [kg*m^2]
	double I_zz = 0.02; // [kg*m^2]
	

	double k_x, a_x, k_y, a_y, k_z, a_z;
	k_x = a_x = k_y = a_y = k_z = a_z = 4.0;

	double k_phi, a_phi, k_teta, a_teta, k_psi, a_psi;
	k_phi = a_phi = k_teta = a_teta = k_psi = a_psi = 16.0;

	
	double dt = 1./200;
	double dt_fwd = 1. / 200; 
	int N = param_N;
	int _N = _param_N;
	//constexpr int N = 10;
	//constexpr int _N = 4;
	ROS_INFO("Steps N equal to %d", N);
	ROS_INFO("Steps _N equal to %d", _N);
	
	double init_val = 0;
	double acc_filter_val_x = init_val;
	double acc_filter_val_y = init_val;
	double acc_filter_val_z = init_val;
	double gyro_filter_val_x = init_val;
	double gyro_filter_val_y = init_val;
	double gyro_filter_val_z = init_val;
	
	double t_prev = ros::Time::now().toSec();
	double T = 0.05; // [s], filter param
	double k_f = 1. / T;
	
	Eigen::Matrix2d F;
	F <<  1, 0, 
			  dt, 1;
			  
	Eigen::Matrix4d _F;
	_F << 	       1,           0,        0,          0, 
				k_f*dt, 1-k_f*dt,        0,          0,
				      dt,           0,        1,          0,
				       0,           0, k_f*dt, 1-k_f*dt;
	
	Eigen::Vector2d Bx {dt / I_xx, 0};
	Eigen::Vector2d By {dt / I_yy, 0};
	Eigen::Vector2d Bz {dt / I_zz, 0};
	
	Eigen::Vector4d _Bx {dt / I_xx, 0, 0, 0};
	Eigen::Vector4d _By {dt / I_yy, 0, 0, 0};
	
	//const double q = param_q;
	Eigen::Matrix2d Q;
	Q << param_qw, 0, 
			  0, param_qa;
			  
	Eigen::Matrix4d _Q;
	_Q << param_qw, 0, 0, 0, 
			  0, param_qw/10, 0, 0,
			  0, 0, param_qa, 0,
			  0, 0, 0, param_qa/10;

	//const double r = param_r;
	Eigen::Matrix2d R;
	R << param_rw, 0, 
			  0, param_ra;
	
	Eigen::Matrix4d _R;
	_R << param_rw, 0, 0, 0, 
			  0, param_rw, 0, 0,
			  0, 0, param_ra, 0,
			  0, 0, 0, param_ra;
	
	const double p = 0.1; 
	Eigen::Matrix2d Px;
	Px << p, 0, 
			  0, p;
	Eigen::Matrix2d Py;
	Py << p, 0, 
			  0, p;
	Eigen::Matrix2d Pz;
	Pz << p, 0, 
			  0, p;

	Eigen::Matrix4d _Px;
	_Px << p, 0, 0, 0,
			  0, p, 0, 0,
			  0, 0, p, 0,
			  0, 0, 0, p;
			  
	Eigen::Matrix4d _Py;
	_Py << p, 0, 0, 0,
			  0, p, 0, 0,
			  0, 0, p, 0,
			  0, 0, 0, p;
			  
	Eigen::Matrix2d H = Eigen::MatrixXd::Identity(2, 2);
	
	Eigen::Matrix4d _H = Eigen::MatrixXd::Identity(4, 4);
	
	//std::cout << "F: " << F << std::endl;
	//std::cout << "Bx: " << Bx << std::endl;
	//std::cout << "R: " << R << std::endl;
	//std::cout << "Q: " << Q << std::endl;
	//std::cout << "Px: " << Px << std::endl;
	
	double u2_kalman = 0;
	double u3_kalman = 0;
	double _u2_kalman = 0;
	double _u3_kalman = 0;


	std::deque<double> w_x_pred_q(N, 0);
	std::deque<double> w_y_pred_q(N, 0);
	std::deque<double> w_z_pred_q(N, 0);
	std::deque<double> _w_x_pred_q(_N, 0);
	std::deque<double> _w_y_pred_q(_N, 0);
	
	std::deque<double> u2_kalman_q(N, 0);
	std::deque<double> u3_kalman_q(N, 0);
	std::deque<double> u4_kalman_q(N, 0);
	std::deque<double> _u2_kalman_q(_N, 0);
	std::deque<double> _u3_kalman_q(_N, 0);

	double w_x_measure, w_x_pred = init_val;
	double phi_measure, phi_pred = init_val;
	double w_y_measure, w_y_pred = init_val;
	double teta_measure, teta_pred = init_val;
	double w_z_measure, w_z_pred = init_val;
	double psi_measure, psi_pred = init_val;
	
	double _w_x_pred = init_val;
	double _phi_pred = init_val;
	double _w_y_pred = init_val;
	double _teta_pred = init_val;
	
	
	double _wm_x_pred = 0, _phi_m_pred = 0,  _wm_y_pred = 0, _teta_m_pred = 0;
	
	double x_cam_prev = 0, y_cam_prev = 0, z_cam_prev = 0;
	
	int count = 0;
	
	while (ros::ok())
	{
		ros::spinOnce();
		
		if (imuUp && cameraUp)
		{
			double dt_filter = glob_angleVel_msg.header.stamp.toSec() - t_prev;
			t_prev = glob_angleVel_msg.header.stamp.toSec();
			
			acc_filter_val_x = acc_filter_val_x - 1/T * (acc_filter_val_x -  glob_angleVel_msg.linear_acceleration.x) * dt_filter;
			acc_filter_val_y = acc_filter_val_y - 1/T * (acc_filter_val_y -  glob_angleVel_msg.linear_acceleration.y) * dt_filter;
			acc_filter_val_z = acc_filter_val_z - 1/T * (acc_filter_val_z -  glob_angleVel_msg.linear_acceleration.z) * dt_filter;
			
			gyro_filter_val_x = gyro_filter_val_x - 1/T * (gyro_filter_val_x -  glob_angleVel_msg.angular_velocity.x) * dt_filter;
			gyro_filter_val_y = gyro_filter_val_y - 1/T * (gyro_filter_val_y -  glob_angleVel_msg.angular_velocity.y) * dt_filter;
			gyro_filter_val_z = gyro_filter_val_z - 1/T * (gyro_filter_val_z -  glob_angleVel_msg.angular_velocity.z) * dt_filter;
			
			sensor_msgs::Imu msg_pf;
			msg_pf.header.stamp = ros::Time::now();
			msg_pf.linear_acceleration.x = acc_filter_val_x;
			msg_pf.linear_acceleration.y = acc_filter_val_y;
			msg_pf.linear_acceleration.z = acc_filter_val_z;
			msg_pf.angular_velocity.x = gyro_filter_val_x;
			msg_pf.angular_velocity.y = gyro_filter_val_y;
			msg_pf.angular_velocity.z = gyro_filter_val_z;
			
			
			//phi_measure = glob_rpy_msg.vector.x; // offset angle
			//phi_measure = std::atan2(glob_angleVel_msg.linear_acceleration.y, glob_angleVel_msg.linear_acceleration.z);
			double phi_offset = 0; // -0.071;
			phi_measure = std::atan2(acc_filter_val_y, acc_filter_val_z) - phi_offset;
			
			//w_x_measure = glob_angleVel_msg.angular_velocity.x;
			w_x_measure = gyro_filter_val_x;
			
			//teta_measure = glob_rpy_msg.vector.y;
			//teta_measure = - std::atan2(glob_angleVel_msg.linear_acceleration.x, glob_angleVel_msg.linear_acceleration.z);
			double teta_offset = 0; // -0.018;
			teta_measure = - std::atan2(acc_filter_val_x, acc_filter_val_z) - teta_offset;
			
			//w_y_measure = glob_angleVel_msg.angular_velocity.y;	
			w_y_measure = gyro_filter_val_y;
			
			//ROS_INFO("acc_x %f acc_y %f acc_z %f gyro_x %f gyro_y %f\n", acc_filter_val_x, acc_filter_val_y, acc_filter_val_z, gyro_filter_val_x, gyro_filter_val_y);
			w_z_measure = gyro_filter_val_z;
			
			geometry_msgs::Vector3 msg_cam_rpy = QuatToEuler(glob_pose_msg.pose.orientation);
			
			double phi_stepped = phi_measure;
			double w_x_stepped = w_x_measure;
			double teta_stepped = teta_measure;
			double w_y_stepped = w_y_measure;
			double psi_stepped = psi_measure;
			double w_z_stepped = w_z_measure;
			
			double _phi_stepped = phi_measure;
			double _w_x_stepped = w_x_measure;
			double _teta_stepped = teta_measure;
			double _w_y_stepped = w_y_measure;
			
			for (int i = 0; i < N; ++i)
			{
				w_x_stepped += (u2_kalman_q[i] / I_xx) * dt_fwd;
				phi_stepped += w_x_pred_q[i] * dt_fwd;
				w_y_stepped  += (u3_kalman_q[i] / I_yy) * dt_fwd;
				teta_stepped += w_y_pred_q[i] * dt_fwd;
				w_z_stepped  += (u4_kalman_q[i] / I_yy) * dt_fwd;
				psi_stepped += w_z_pred_q[i] * dt_fwd;
			}
			// NEW
			for (int i = 0; i < _N; ++i)
			{
				_w_x_stepped += (_u2_kalman_q[i] / I_xx) * dt_fwd;
				_phi_stepped += _w_x_pred_q[i] * dt_fwd;
				_w_y_stepped  += (_u3_kalman_q[i] / I_yy) * dt_fwd;
				_teta_stepped += _w_y_pred_q[i] * dt_fwd;
			}
			
			
			sensor_msgs::Imu msg_st;
			msg_st.header.stamp = ros::Time::now();
			msg_st.angular_velocity.x = w_x_stepped;
			msg_st.angular_velocity.y = w_y_stepped;
			msg_st.angular_velocity.z = w_z_stepped;
			//NEW
			sensor_msgs::Imu _msg_st;
			_msg_st.header.stamp = ros::Time::now();
			_msg_st.angular_velocity.x = _w_x_stepped;
			_msg_st.angular_velocity.y = _w_y_stepped;
			
			geometry_msgs::Vector3Stamped msg_rpy_stepped;
			msg_rpy_stepped.header.stamp = ros::Time::now();
			msg_rpy_stepped.vector.x = phi_stepped;
			msg_rpy_stepped.vector.y = teta_stepped;
			msg_rpy_stepped.vector.z = psi_stepped;			
			//NEW
			geometry_msgs::Vector3Stamped _msg_rpy_stepped;
			_msg_rpy_stepped.header.stamp = ros::Time::now();
			_msg_rpy_stepped.vector.x = _phi_stepped;
			_msg_rpy_stepped.vector.y = _teta_stepped;
			
			Eigen::Vector2d Xx_k_prev {w_x_pred, phi_pred};
			Eigen::Vector2d Xx_k = F * Xx_k_prev + Bx * u2_kalman;
			Eigen::Vector2d Xy_k_prev {w_y_pred, teta_pred};
			Eigen::Vector2d Xy_k = F * Xy_k_prev + By * u3_kalman;
			Eigen::Vector2d Xz_k_prev {w_z_pred, psi_pred};
			Eigen::Vector2d Xz_k = F * Xz_k_prev + Bz * u2_kalman;
			// NEW
			Eigen::Vector4d _Xx_k_prev {_w_x_pred, _wm_x_pred, _phi_pred, _phi_m_pred};
			Eigen::Vector4d _Xx_k = _F * _Xx_k_prev + _Bx * _u2_kalman;
			Eigen::Vector4d _Xy_k_prev {_w_y_pred, _wm_x_pred, teta_pred, _teta_m_pred};
			Eigen::Vector4d _Xy_k = _F * _Xy_k_prev + _By * _u3_kalman;
			
			Px = F * Px * F.transpose() + Q;
			Py = F * Py * F.transpose() + Q;
			Pz = F * Pz * F.transpose() + Q;
			
			// NEW
			_Px = _F * _Px * _F.transpose() + _Q;
			_Py = _F * _Py * _F.transpose() + _Q;
			
			
			Eigen::Vector2d Zx_k {w_x_stepped, phi_stepped};
			Eigen::Vector2d Yx_k = Zx_k - H * Xx_k;
			Eigen::Vector2d Zy_k {w_y_stepped, teta_stepped};
			Eigen::Vector2d Yy_k = Zy_k - H * Xy_k;
			Eigen::Vector2d Zz_k {w_z_stepped, psi_stepped};
			Eigen::Vector2d Yz_k = Zz_k - H * Xz_k;
			// NEW
			Eigen::Vector4d _Zx_k {_w_x_stepped, w_x_measure, _phi_stepped, phi_measure};
			Eigen::Vector4d _Yx_k = _Zx_k - _H * _Xx_k;
			Eigen::Vector4d _Zy_k {_w_y_stepped, w_y_measure, _teta_stepped, teta_measure};
			Eigen::Vector4d _Yy_k = _Zy_k - _H * _Xy_k;
			
			Eigen::Matrix2d Sx_k = H * Px * H.transpose() + R;
			Eigen::Matrix2d Kx_k = Px * H.transpose() * Sx_k.inverse();
			Eigen::Matrix2d Sy_k = H * Py * H.transpose() + R;
			Eigen::Matrix2d Ky_k = Py * H.transpose() * Sy_k.inverse();
			Eigen::Matrix2d Sz_k = H * Pz * H.transpose() + R;
			Eigen::Matrix2d Kz_k = Pz * H.transpose() * Sz_k.inverse();
			//NEW
			Eigen::Matrix4d _Sx_k = _H * _Px * _H.transpose() + _R;
			Eigen::Matrix4d _Kx_k = _Px * _H.transpose() * _Sx_k.inverse();
			Eigen::Matrix4d _Sy_k = _H * _Py * _H.transpose() + _R;
			Eigen::Matrix4d _Ky_k = _Py * _H.transpose() * _Sy_k.inverse();
			
			Xx_k += Kx_k * Yx_k;
			Px = (Eigen::MatrixXd::Identity(2, 2) - Kx_k * H) * Px;
			Xy_k += Ky_k * Yy_k;
			Py = (Eigen::MatrixXd::Identity(2, 2) - Ky_k * H) * Py;
			Xz_k += Kz_k * Yz_k;
			Pz = (Eigen::MatrixXd::Identity(2, 2) - Kz_k * H) * Pz;
			//NEW
			_Xx_k += _Kx_k * _Yx_k;
			_Px = (Eigen::MatrixXd::Identity(4, 4) - _Kx_k * _H) * _Px;
			_Xy_k += _Ky_k * _Yy_k;
			_Py = (Eigen::MatrixXd::Identity(4, 4) - _Ky_k * _H) * _Py;
			
			w_x_pred = Xx_k(0);
			phi_pred = Xx_k(1);
			w_y_pred = Xy_k(0);
			teta_pred = Xy_k(1);
			w_z_pred = Xz_k(0);
			psi_pred = Xz_k(1);
			
			_w_x_pred = _Xx_k(0);
			_wm_x_pred = _Xx_k(1);
			_phi_pred = _Xx_k(2);
			_phi_m_pred = _Xx_k(3);
			_w_y_pred = _Xy_k(0);
			_wm_y_pred = _Xy_k(1);
			_teta_pred = _Xy_k(2);
			_teta_m_pred = _Xy_k(3);
			
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
			
			double u1 = sqrt(H_xx*H_xx + H_yy*H_yy + H_zz*H_zz) * 0; // ZERO
			if (u1 >= 8)
				u1 = 8;
			//double u1 = m * g;
			double u2 = I_xx * (-(a_phi+k_phi)*w_x_pred - a_phi*k_phi*(phi_pred-phi_ref)) * 0; // ZERO
			double u3 = I_yy * (-(a_teta+k_teta)*w_y_pred - a_teta*k_teta*(teta_pred-teta_ref)) * 0; // ZERO
			
			
			double _u1 = m * g;
			double _u2 = I_xx * (-(a_phi+k_phi)*_w_x_pred - a_phi*k_phi*_phi_pred);
			double _u3 = I_yy * (-(a_teta+k_teta)*_w_y_pred - a_teta*k_teta*_teta_pred);
			/// TEST CONTROLLER
			//double u2 = I_xx * (-(a_phi+k_phi)*w_x_measure - a_phi*k_phi*phi_measure);
			//double u3 = I_yy * (-(a_teta+k_teta)*w_y_measure - a_teta*k_teta*teta_measure);
			
			/// Debug
			//u3 = glob_angleVel_msg.angular_velocity.y > 6. ? 6. : 0;
			//if (u3 > 5)
			//	ROS_INFO("msg: %f u3: %f\n", glob_angleVel_msg.angular_velocity.y, u3); 
			
			u2_kalman = u2;
			u3_kalman = u3;
			//NEW
			_u2_kalman = _u2;
			_u3_kalman = _u3;
			
			u2_kalman_q.pop_front();
			u3_kalman_q.pop_front();
			u2_kalman_q.push_back(u2);
			u3_kalman_q.push_back(u3);
			//NEW
			_u2_kalman_q.pop_front();
			_u3_kalman_q.pop_front();
			_u2_kalman_q.push_back(_u2);
			_u3_kalman_q.push_back(_u3);

			w_x_pred_q.pop_front();
			w_y_pred_q.pop_front();
			w_x_pred_q.push_back(w_x_pred);
			w_y_pred_q.push_back(w_y_pred);
			//NEW
			_w_x_pred_q.pop_front();
			_w_y_pred_q.pop_front();
			_w_x_pred_q.push_back(_w_x_pred);
			_w_y_pred_q.push_back(_w_y_pred);
			
			//for (int i=0; i<_N; ++i)
			//	ROS_INFO("_u2[%d]=%f ", i, _u2_kalman_q[i]);
			//ROS_INFO("_u2_kalman %f u2_kalman %f\n", _u2_kalman, u2_kalman);

			//ROS_INFO("w_x_stepped=%f, _w_x_stepped=%f, w_x_pred=%f, _w_x_pred=%f,  phi_stepped=%f, _phi_stepped=%f, phi_pred=%f, _phi_pred=%f, u2=%f, _u2=%f", 
			//	w_x_stepped, _w_x_stepped, w_x_pred, _w_x_pred, phi_stepped, _phi_stepped, phi_pred, _phi_pred, u2, _u2);


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
			msg.point.x = u1; //u1
			msg.point.y = u2; //u2
			msg.point.z = u3; //u3
				
			
			sensor_msgs::Imu msg_mod = glob_angleVel_msg;
			msg_mod.header.stamp = ros::Time::now(); //wasn't present
			msg_mod.angular_velocity.x = w_x_pred;
			msg_mod.angular_velocity.y = w_y_pred;
			msg_mod.angular_velocity.z = w_z_pred;
			//NEW
			sensor_msgs::Imu _msg_mod = glob_angleVel_msg;
			_msg_mod.header.stamp = ros::Time::now();
			_msg_mod.angular_velocity.x = _w_x_pred;
			_msg_mod.angular_velocity.y = _w_y_pred;
			
			geometry_msgs::Vector3Stamped msg_rpy_pred;
			msg_rpy_pred.header.stamp = ros::Time::now();
			msg_rpy_pred.vector.x = phi_pred;
			msg_rpy_pred.vector.y = teta_pred;
			msg_rpy_pred.vector.z = psi_pred;
			//NEW
			geometry_msgs::Vector3Stamped _msg_rpy_pred;
			_msg_rpy_pred.header.stamp = ros::Time::now();
			_msg_rpy_pred.vector.x = _phi_pred;
			_msg_rpy_pred.vector.y = _teta_pred;
			_msg_rpy_pred.vector.z = 0;
		
		
			chatter_pub.publish(msg);
			
			chatter_imu_prefilter.publish(msg_pf);
			chatter_imu_stepped.publish(msg_st);
			_chatter_imu_stepped.publish(_msg_st);
			chatter_rpy_stepped.publish(msg_rpy_stepped);
			_chatter_rpy_stepped.publish(_msg_rpy_stepped);
			
			chatter_pub_mod.publish(msg_mod);
			_chatter_pub_mod.publish(_msg_mod);
			chatter_rpy_pred.publish(msg_rpy_pred);
			_chatter_rpy_pred.publish(_msg_rpy_pred);
			
			pub_cam_rpy.publish(msg_cam_rpy);
			
			++count;
			
		}
		loop_rate.sleep();
	}

	return 0;
}
