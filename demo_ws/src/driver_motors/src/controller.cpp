#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#include <vector>
#include <queue>
#include <deque>
#include <sstream>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <fstream>


geometry_msgs::Vector3Stamped glob_rpy_msg; 
sensor_msgs::Imu glob_angleVel_msg;

//double remote_u1 = 0;
//double remote_u2 = 0;
//double remote_u3 = 0;

bool imuUp = false;

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

/*
void remoteCallback(const geometry_msgs::Point& msg)
{
	remote_u1 = msg.x;
	remote_u2 = msg.y;
	remote_u3 = msg.z;
}
*/


int main(int argc, char **argv)
{	
	
	ros::init(argc, argv, "controller");
	ros::NodeHandle n("~");

	int param_N, _param_N;
	double param_rw, param_ra;
	double param_qw, param_qa;

	n.param<int>("N", param_N, 50);
	n.param<int>("newN", _param_N, 4);
	n.param<double>("rw", param_rw, 0.1);
	n.param<double>("qw", param_qw, 0.001);
	n.param<double>("ra", param_ra, 0.1);
	n.param<double>("qa", param_qa, 0.001);
	
	ROS_INFO("velocity 'r' equal to %f", param_rw);
	ROS_INFO("velocity 'q' equal to %f", param_qw);
	ROS_INFO("angle 'r' equal to %f", param_ra);
	ROS_INFO("angle 'q' equal to %f", param_qa);
	
	
	std::ofstream dr_file;
	std::ofstream dpf_file;
	std::ofstream _dm_file;
	std::ofstream _u_file;
	dr_file.open("dr_13_04_model_phi1e-1.csv");
	dpf_file.open("dpf_13_04_model_phi1e-1.csv");
	_dm_file.open("_dm_13_04_model_phi1e-1.csv");
	_u_file.open("_u_13_04_model_phi1e-1.csv");
	
	ros::Subscriber subRPY = n.subscribe("/imu/rpy/filtered", 1, rpyCallback);
	ros::Subscriber subAngleVel = n.subscribe("/imu/data_raw", 1, angleVelCallback); //with filter subscribe on /imu/data!
	//ros::Subscriber subRemote = n.subscribe("/remote", 1, remoteCallback);
	
	ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("/controller/data_raw", 1);

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::PointStamped>("/controls", 1);
	ros::Publisher _chatter_pub = n.advertise<geometry_msgs::PointStamped>("/_controls", 1);
	ros::Publisher chatter_imu_prefilter = n.advertise<sensor_msgs::Imu>("/imu/data_prefiltered", 1);
	ros::Publisher chatter_rpy_prefilter = n.advertise<geometry_msgs::Vector3Stamped>("/imu/rpy/prefiltered", 1);
	
	ros::Publisher chatter_imu_stepped = n.advertise<sensor_msgs::Imu>("/imu/data_stepped", 1);
	ros::Publisher chatter_rpy_stepped = n.advertise<geometry_msgs::Vector3Stamped>("/imu/rpy/stepped", 1);
	ros::Publisher _chatter_imu_stepped = n.advertise<sensor_msgs::Imu>("/imu/_data_stepped", 1);
	ros::Publisher _chatter_rpy_stepped = n.advertise<geometry_msgs::Vector3Stamped>("/imu/rpy/_stepped", 1);

	ros::Publisher chatter_pub_mod = n.advertise<sensor_msgs::Imu>("/imu/data_mod", 1);
	ros::Publisher chatter_rpy_pred = n.advertise<geometry_msgs::Vector3Stamped>("/imu/rpy/predicted", 1);
	ros::Publisher _chatter_pub_mod = n.advertise<sensor_msgs::Imu>("/imu/_data_mod", 1);
	ros::Publisher _chatter_rpy_pred = n.advertise<geometry_msgs::Vector3Stamped>("/imu/rpy/_predicted", 1);
	
	
	ros::Rate loop_rate(200);

	// From article
	double m = 0.4; // [kg]
	double g = 9.81; // [m/c^2]

	double I_xx = 0.02; //0.0464; // [kg*m^2] or 0.01
	double I_yy = 0.02; //0.0464; // [kg*m^2] or 0.01

	double k_x, a_x, k_y, a_y, k_z, a_z;
	k_x = a_x = k_y = a_y = k_z = a_z = 4.0;

	double k_teta, a_teta, k_phi, a_phi;
	k_teta = a_teta = k_phi = a_phi = 16.0;
	
	double dt = 1./200;
	double dt_fwd = 1. / 200; 
	int N = param_N;
	int _N = _param_N;
	ROS_INFO("Steps equal to %d", N);
	ROS_INFO("New steps equal to %d", _N);
	
	double acc_filter_val_x = 0;
	double acc_filter_val_y = 0;
	double acc_filter_val_z = 0;
	double gyro_filter_val_x = 0;
	double gyro_filter_val_y = 0;
	
	double t_prev = ros::Time::now().toSec();
	double T = 0.05; // [s], filter param
	double k_f = 1. / T;
	
	double w_x_measure;
	double phi_measure;
	double w_y_measure;
	double teta_measure;
	double psi_measure;
	/*
	Eigen::Matrix2d F;
	F <<  1, 0, 
			  dt, 1;
			  
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
	
	double u2_kalman = 0;
	double u3_kalman = 0;


	std::deque<double> w_x_pred_q(N, 0);
	std::deque<double> w_y_pred_q(N, 0);
	
	std::deque<double> u2_kalman_q(N, 0);
	std::deque<double> u3_kalman_q(N, 0);
	*/
	
	///ALTERNATIVE
	Eigen::Matrix4d _F;
	_F << 	       1,           0,        0,          0, 
				k_f*dt, 1-k_f*dt,        0,          0,
				      dt,           0,        1,          0,
				       0,           0, k_f*dt, 1-k_f*dt;

	Eigen::Vector4d _Bx {dt / I_xx, 0, 0, 0};
	Eigen::Vector4d _By {dt / I_yy, 0, 0, 0};
	
	Eigen::Matrix4d _Q;
	_Q << param_qw, 0, 0, 0, 
			  0, param_qw, 0, 0,
			  0, 0, param_qa, 0,
			  0, 0, 0, param_qa;
	
	Eigen::Matrix4d _R;
	_R << param_rw, 0, 0, 0, 
			  0, param_rw/10, 0, 0,
			  0, 0, param_ra, 0,
			  0, 0, 0, param_ra/10;
	
	const double _p = 0.1; 
	Eigen::Matrix4d _Px;
	_Px << _p, 0, 0, 0,
			  0, _p, 0, 0,
			  0, 0, _p, 0,
			  0, 0, 0, _p;
	Eigen::Matrix4d _Py;
	_Py << _p, 0, 0, 0,
			  0, _p, 0, 0,
			  0, 0, _p, 0,
			  0, 0, 0, _p;
			  
	Eigen::Matrix4d _H = Eigen::MatrixXd::Identity(4, 4);
	
	int _delay_N = 4;
	std::deque<double> _phi_delay_q(_delay_N, 0);
	std::deque<double> _w_x_delay_q(_delay_N, 0);
	std::deque<double> _teta_delay_q(_delay_N, 0);
	std::deque<double> _w_y_delay_q(_delay_N, 0);

	std::deque<double> _u2_kalman_q(_N, 0);
	double _u2_kalman = 0;
	double _w_x_pred = 0;
	double _w_x_pf_pred = 0;
	double _phi_pred = 0;
	double _phi_pf_pred = 0;
	 
	std::deque<double> _u3_kalman_q(_N, 0);
	double _u3_kalman = 0;
	double _w_y_pred = 0;
	double _w_y_pf_pred = 0;
	double _teta_pred = 0;
	double _teta_pf_pred = 0;
	
	// PYTHON
	double w_x = 0.;
	double phi = 0.;
	double w_y = 0.;
	double teta = 0.;
	double psi = 0.;
	
	double _w_x_stepped, _w_x_pf_stepped, _phi_stepped, _phi_pf_stepped;
	double _w_y_stepped, _w_y_pf_stepped, _teta_stepped, _teta_pf_stepped;
	double _u1, _u2, _u3;

	int count = 0;
	while (ros::ok())
	{
		ros::spinOnce();
		
		double dt_filter = ros::Time::now().toSec() - t_prev;
		t_prev = ros::Time::now().toSec();

		//double dt_filter = dt; // FOR PYTHON CODE TESTING

		if (imuUp)
		{
			
			bool modelOn = true;
			
			//ROS_INFO("ITERATION %d", count);
			
			//phi += w_x * dt;
			//w_x += (_u2_kalman / I_xx) * dt;
			//dr_file << ros::Time::now() << "," << w_x << "," << phi << "\n";
			//ROS_INFO("w_x=%.8f phi=%.8f", w_x, phi);
			w_x = glob_angleVel_msg.angular_velocity.x;
			w_y = glob_angleVel_msg.angular_velocity.y;
			phi = glob_angleVel_msg.linear_acceleration.x;
			teta = glob_angleVel_msg.linear_acceleration.y;
			psi = glob_angleVel_msg.linear_acceleration.z;
			dr_file << glob_angleVel_msg.header.stamp << "," << w_x << "," << w_y << "," << phi << "," << teta << "," << psi << "\n";

			// DELAY	
			phi_measure = _phi_delay_q.front();
			w_x_measure = _w_x_delay_q.front();
			_phi_delay_q.pop_front();
			_w_x_delay_q.pop_front();
			_phi_delay_q.push_back(phi);
			_w_x_delay_q.push_back(w_x);
			
			teta_measure = _teta_delay_q.front();
			w_y_measure = _w_y_delay_q.front();
			_teta_delay_q.pop_front();
			_w_y_delay_q.pop_front();
			_teta_delay_q.push_back(teta);
			_w_y_delay_q.push_back(w_y);

			// NO DELAY
			//w_x_measure = w_x;
			//w_y_measure = w_y;
			//phi_measure = phi;
			//teta_measure = teta;
			//psi_measure = psi;

			//ROS_INFO("w_x_measure %f", w_x_measure);
			//for (int j = 0; j < _delay_N; ++j)
			//	ROS_INFO("_w_delay_q[%d]=%f", j, _w_x_delay_q[j]);

			// PREFILTER
			acc_filter_val_x = acc_filter_val_x - 1/T * (acc_filter_val_x -  phi_measure) * dt_filter;
			gyro_filter_val_x = gyro_filter_val_x - 1/T * (gyro_filter_val_x -  w_x_measure) * dt_filter;
			
			acc_filter_val_y = acc_filter_val_y - 1/T * (acc_filter_val_y -  teta_measure) * dt_filter;
			gyro_filter_val_y = gyro_filter_val_y - 1/T * (gyro_filter_val_y -  w_y_measure) * dt_filter;
			dpf_file << ros::Time::now() << "," << gyro_filter_val_x << "," << gyro_filter_val_y << "," << acc_filter_val_x << "," << acc_filter_val_y << "," << acc_filter_val_z << "\n";
			
			acc_filter_val_z = acc_filter_val_z - 1/T * (acc_filter_val_y -  teta_measure) * dt_filter;
			//ROS_INFO("w_x_pf=%.8f phi_pf=%.8f", gyro_filter_val_x, acc_filter_val_x);

			double w_x_pf = gyro_filter_val_x;
			double w_y_pf = gyro_filter_val_y;

			double phi_pf, teta_pf;
			if (modelOn)
			{
				phi_pf = acc_filter_val_x;
				teta_pf = acc_filter_val_y;
			} else
			{
				if (psi > 20)
					psi = 20;
				else if (psi < 4)
					psi = 4;	
				
				phi_measure = std::atan2(teta, psi);
				teta_measure = - std::atan2(phi, psi);
				phi_pf = std::atan2(acc_filter_val_y, acc_filter_val_z);
				teta_pf = - std::atan2(acc_filter_val_x, acc_filter_val_z);
			}
			// STEP
			_w_x_stepped = w_x_measure;
			_w_x_pf_stepped = w_x_pf;
			_phi_stepped = phi_measure;
			_phi_pf_stepped = phi_pf;
			
			_w_y_stepped = w_y_measure;
			_w_y_pf_stepped = w_y_pf;
			_teta_stepped = teta_measure;
			_teta_pf_stepped = teta_pf;
			
			for (int i = 0; i < _N; ++i)
			{
				_phi_stepped += _w_x_stepped * dt_fwd;
				_w_x_stepped += (_u2_kalman_q[i] / I_xx) * dt_fwd;
				
				_teta_stepped += _w_y_stepped * dt_fwd;
				_w_y_stepped += (_u3_kalman_q[i] / I_yy) * dt_fwd;
			}
			
			// KALMAN FILTER-4
			/// x-axis
			Eigen::Vector4d _Xx_k_prev {_w_x_pred, _w_x_pf_pred, _phi_pred, _phi_pf_pred};
			Eigen::Vector4d _Xx_k = _F * _Xx_k_prev + _Bx * _u2_kalman;
			//ROS_INFO_STREAM("_Xx_k_prev\n" << _Xx_k_prev << " \n_Xx_k\n" << _Xx_k << "\n");

			_Px = _F * _Px * _F.transpose() + _Q;
			//ROS_INFO_STREAM("_Px\n" << _Px  << "\n");

			Eigen::Vector4d _Zx_k {_w_x_stepped, _w_x_pf_stepped, _phi_stepped, _phi_pf_stepped};
			Eigen::Vector4d _Yx_k = _Zx_k - _H * _Xx_k;
			//ROS_INFO_STREAM("_Zx_k\n" << _Zx_k << " \n_Yx_k\n" << _Yx_k << "\n");

			Eigen::Matrix4d _Sx_k = _H * _Px * _H.transpose() + _R;
			Eigen::Matrix4d _Kx_k = _Px * _H.transpose() * _Sx_k.inverse();
			//ROS_INFO_STREAM("_Sx_k\n" << _Sx_k << "\n_Kx_k\n" << _Kx_k << "\n");

			_Xx_k += _Kx_k * _Yx_k;
			_Px = (Eigen::MatrixXd::Identity(4, 4) - _Kx_k * _H) * _Px;
			//ROS_INFO_STREAM(" _Xx_k(new)\n" << _Xx_k << "\n");
			//ROS_INFO_STREAM("_Px(new)\n" << _Px << "\n");

			_w_x_pred = _Xx_k(0);
			_w_x_pf_pred = _Xx_k(1);
			_phi_pred = _Xx_k(2);
			_phi_pf_pred = _Xx_k(3);
			//ROS_INFO("_w_x_pred=%.8f _phi_pred=%.8f", _w_x_pred, _phi_pred);

			/// y-axis
			Eigen::Vector4d _Xy_k_prev {_w_y_pred, _w_y_pf_pred, _teta_pred, _teta_pf_pred};
			Eigen::Vector4d _Xy_k = _F * _Xy_k_prev + _By * _u3_kalman;

			_Py = _F * _Py * _F.transpose() + _Q;

			Eigen::Vector4d _Zy_k {_w_y_stepped, _w_y_pf_stepped, _teta_stepped, _teta_pf_stepped};
			Eigen::Vector4d _Yy_k = _Zy_k - _H * _Xy_k;

			Eigen::Matrix4d _Sy_k = _H * _Py * _H.transpose() + _R;
			Eigen::Matrix4d _Ky_k = _Py * _H.transpose() * _Sy_k.inverse();

			_Xy_k += _Ky_k * _Yy_k;
			_Py = (Eigen::MatrixXd::Identity(4, 4) - _Ky_k * _H) * _Py;

			_w_y_pred = _Xy_k(0);
			_w_y_pf_pred = _Xy_k(1);
			_teta_pred = _Xy_k(2);
			_teta_pf_pred = _Xy_k(3);
			_dm_file << ros::Time::now() << "," << _w_x_pred << "," << _w_y_pred << "," << _phi_pred << "," << _teta_pred << "\n";
			
			// CONTROLS
			double phi_ref = 0, teta_ref = 0;
			_u1 = m * g;
			_u2 = I_xx * (-(a_phi+k_phi)*_w_x_pred - a_phi*k_phi*(_phi_pred-phi_ref));
			_u3 = I_yy * (-(a_teta+k_teta)*_w_y_pred - a_teta*k_teta*(_teta_pred-teta_ref));
			
			_u2_kalman = _u2;
			_u3_kalman = _u3;
			//ROS_INFO("_u2=%.8f", _u2);

			_u_file << ros::Time::now() << "," << _u1 << "," << _u2 << "," << _u3 << "\n";
			
			if (_N != 0)
			{
				_u2_kalman_q.pop_front();
				_u2_kalman_q.push_back(_u2);
				
				_u3_kalman_q.pop_front();
				_u3_kalman_q.push_back(_u3);
			}
			
			// MESSAGES
			sensor_msgs::Imu msg_imu;
			msg_imu.header.stamp = ros::Time::now();
			msg_imu.angular_velocity.x = w_x_measure;
			msg_imu.linear_acceleration.x = phi_measure;
			msg_imu.angular_velocity.y = w_y_measure;
			msg_imu.linear_acceleration.y = teta_measure;
			
			geometry_msgs::PointStamped msg;
			msg.header.stamp = ros::Time::now();
			msg.point.x = _u1;
			msg.point.y = _u2;
			msg.point.z = _u3;
			
			sensor_msgs::Imu msg_pf;
			msg_pf.header.stamp = ros::Time::now();
			msg_pf.linear_acceleration.x = acc_filter_val_x;
			msg_pf.angular_velocity.x = gyro_filter_val_x;
			msg_pf.linear_acceleration.y = acc_filter_val_y;
			msg_pf.angular_velocity.y = gyro_filter_val_y;
			
			sensor_msgs::Imu _msg_st;
			_msg_st.header.stamp = ros::Time::now();
			_msg_st.angular_velocity.x = _w_x_stepped;
			_msg_st.angular_velocity.y = _w_y_stepped;
			
			geometry_msgs::Vector3Stamped _msg_rpy_stepped;
			_msg_rpy_stepped.header.stamp = ros::Time::now();
			_msg_rpy_stepped.vector.x = _phi_stepped;
			_msg_rpy_stepped.vector.y = _teta_stepped;
			
			sensor_msgs::Imu _msg_mod;
			_msg_mod.header.stamp = ros::Time::now();
			_msg_mod.angular_velocity.x = _w_x_pred;
			_msg_mod.angular_velocity.y = _w_y_pred;
			
			geometry_msgs::Vector3Stamped _msg_rpy_pred;
			_msg_rpy_pred.header.stamp = ros::Time::now();
			_msg_rpy_pred.vector.x = _phi_pred;
			_msg_rpy_pred.vector.y = _teta_pred;
			_msg_rpy_pred.vector.z = 0;
			
			// PUBLISHERS
			pub_imu.publish(msg_imu);
			
			chatter_pub.publish(msg);
			chatter_imu_prefilter.publish(msg_pf);
			_chatter_imu_stepped.publish(_msg_st);
			_chatter_rpy_stepped.publish(_msg_rpy_stepped);
		
			_chatter_pub_mod.publish(_msg_mod);
			_chatter_rpy_pred.publish(_msg_rpy_pred);
			
			++count;
		}
		/*
		double dt_filter = glob_angleVel_msg.header.stamp.toSec() - t_prev;
		t_prev = glob_angleVel_msg.header.stamp.toSec();

		
		// REMOVE THROWS IN ANGLE 4KALMAN
		if (glob_angleVel_msg.linear_acceleration.z > 20)
			glob_angleVel_msg.linear_acceleration.z = 20;
		else if (glob_angleVel_msg.linear_acceleration.z < 4)
			glob_angleVel_msg.linear_acceleration.z = 4;	
		
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
		
		
		//w_x_measure = glob_angleVel_msg.angular_velocity.x;
		w_x_measure = gyro_filter_val_x;
		
		//w_y_measure = glob_angleVel_msg.angular_velocity.y;	
		w_y_measure = gyro_filter_val_y;
		
		// REMOVE THROWS IN ANGLE
		if (acc_filter_val_z > 20)
			acc_filter_val_z = 20;
		else if (acc_filter_val_z < 4)
			acc_filter_val_z = 4;
		
		double phi_offset = 0, teta_offset = 0;
		bool modelOn = true;
		
		if (modelOn)
		{
			phi_measure = acc_filter_val_x; // model angle_x
			teta_measure = acc_filter_val_y;
		} else
		{
			//phi_measure = glob_rpy_msg.vector.x; // offset angle
			//phi_measure = std::atan2(glob_angleVel_msg.linear_acceleration.y, glob_angleVel_msg.linear_acceleration.z);
			phi_measure = std::atan2(acc_filter_val_y, acc_filter_val_z) - phi_offset;
			
			//teta_measure = glob_rpy_msg.vector.y;
			//teta_measure = - std::atan2(glob_angleVel_msg.linear_acceleration.x, glob_angleVel_msg.linear_acceleration.z);
			teta_measure = - std::atan2(acc_filter_val_x, acc_filter_val_z) - teta_offset;
			
			//ROS_INFO("acc_x %f acc_y %f acc_z %f gyro_x %f gyro_y %f\n", acc_filter_val_x, acc_filter_val_y, acc_filter_val_z, gyro_filter_val_x, gyro_filter_val_y);
		}
		
		geometry_msgs::Vector3Stamped msg_rpy_pf;
		msg_rpy_pf.header.stamp = ros::Time::now();
		msg_rpy_pf.vector.x = phi_measure;
		msg_rpy_pf.vector.y = teta_measure;
		msg_rpy_pf.vector.z = 0;
		
		

		double phi_stepped = phi_measure;
		double w_x_stepped = w_x_measure;
		
		double teta_stepped = teta_measure;
		double w_y_stepped = w_y_measure;
		
		for (int i = 0; i < N; ++i)
		{
			w_x_stepped += (u2_kalman_q[i] / I_xx) * dt_fwd;
			phi_stepped += w_x_pred_q[i] * dt_fwd; //phi_stepped += w_x_stepped * dt_fwd;
			w_y_stepped  += (u3_kalman_q[i] / I_yy) * dt_fwd;
			teta_stepped += w_y_pred_q[i] * dt_fwd; //teta_stepped += w_y_stepped * dt_fwd;
		}
		
		///ALTERNATIVE
		
		double _w_x_stepped = glob_angleVel_msg.angular_velocity.x;
		double _w_x_pf_stepped = w_x_measure;
		double _phi_stepped = modelOn ? glob_angleVel_msg.linear_acceleration.x : std::atan2(glob_angleVel_msg.linear_acceleration.y, glob_angleVel_msg.linear_acceleration.z); //???
		double _phi_pf_stepped = phi_measure;
		
		double _w_y_stepped = glob_angleVel_msg.angular_velocity.y;
		double _w_y_pf_stepped = w_y_measure;
		double _teta_stepped = modelOn ? glob_angleVel_msg.linear_acceleration.y : - std::atan2(glob_angleVel_msg.linear_acceleration.x, glob_angleVel_msg.linear_acceleration.z);
		double _teta_pf_stepped = teta_measure; 
		
		for (int i = 0; i < _N; ++i)
		{
			_phi_stepped += _w_x_stepped * dt_fwd;
			_w_x_stepped += (_u2_kalman_q[i] / I_xx) * dt_fwd;
			_teta_stepped += _w_y_stepped * dt_fwd;
			_w_y_stepped += (_u3_kalman_q[i] / I_yy) * dt_fwd;
		}
		
		
		
		sensor_msgs::Imu msg_st;
		msg_st.header.stamp = ros::Time::now();
		msg_st.angular_velocity.x = w_x_stepped;
		msg_st.angular_velocity.y = w_y_stepped;
		geometry_msgs::Vector3Stamped msg_rpy_stepped;
		msg_rpy_stepped.header.stamp = ros::Time::now();
		msg_rpy_stepped.vector.x = phi_stepped;
		msg_rpy_stepped.vector.y = teta_stepped;
		///ALTERNATIVE
		sensor_msgs::Imu _msg_st;
		_msg_st.header.stamp = ros::Time::now();
		_msg_st.angular_velocity.x = _w_x_stepped;
		_msg_st.angular_velocity.y = _w_y_stepped;
		geometry_msgs::Vector3Stamped _msg_rpy_stepped;
		_msg_rpy_stepped.header.stamp = ros::Time::now();
		_msg_rpy_stepped.vector.x = _phi_stepped;
		_msg_rpy_stepped.vector.y = _teta_stepped;
		
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
		
		///ALTERNATIVE
		
		Eigen::Vector4d _Xx_k_prev {_w_x_pred, _w_x_pf_pred, _phi_pred, _phi_pf_pred};
		Eigen::Vector4d _Xx_k = _F * _Xx_k_prev + _Bx * _u2_kalman;
		
		_Px = _F * _Px * _F.transpose() + _Q;
		
		Eigen::Vector4d _Zx_k {_w_x_stepped, _w_x_pf_stepped, _phi_stepped, _phi_pf_stepped};
		Eigen::Vector4d _Yx_k = _Zx_k - _H * _Xx_k;
		
		Eigen::Matrix4d _Sx_k = _H * _Px * _H.transpose() + _R;
		Eigen::Matrix4d _Kx_k = _Px * _H.transpose() * _Sx_k.inverse();
		
		_Xx_k += _Kx_k * _Yx_k;
		_Px = (Eigen::MatrixXd::Identity(4, 4) - _Kx_k * _H) * _Px;
		
		_w_x_pred = _Xx_k(0);
		_w_x_pf_pred = _Xx_k(1);
		_phi_pred = _Xx_k(2);
		_phi_pf_pred = _Xx_k(3);
		 
		Eigen::Vector4d _Xy_k_prev {_w_y_pred, _w_y_pf_pred, _teta_pred, _teta_pf_pred};
		Eigen::Vector4d _Xy_k = _F * _Xy_k_prev + _By * _u3_kalman;
		
		_Py = _F * _Py * _F.transpose() + _Q;
		
		Eigen::Vector4d _Zy_k {_w_y_stepped, _w_y_pf_stepped, _teta_stepped, _teta_pf_stepped};
		Eigen::Vector4d _Yy_k = _Zy_k - _H * _Xy_k;
		
		Eigen::Matrix4d _Sy_k = _H * _Py * _H.transpose() + _R;
		Eigen::Matrix4d _Ky_k = _Py * _H.transpose() * _Sy_k.inverse();
		
		_Xy_k += _Ky_k * _Yy_k;
		_Py = (Eigen::MatrixXd::Identity(4, 4) - _Ky_k * _H) * _Py;
		
		_w_y_pred = _Xy_k(0);
		_w_y_pf_pred = _Xy_k(1);
		_teta_pred = _Xy_k(2);
		_teta_pf_pred = _Xy_k(3);
		
		
		
		
		double x_cam = 0, y_cam = 0, z_cam = 0;
		double dx_cam = 0, dy_cam = 0, dz_cam = 0;
		
		double H_xx = m * (-(a_x+k_x)*dx_cam - a_x*k_x*x_cam);
		double H_yy = m * (-(a_y+k_y)*dy_cam - a_y*k_y*y_cam);
		double H_zz = m * (-(a_z+k_z)*dz_cam - a_z*k_z*z_cam);
		
		//double teta_ref = std::atan2(H_xx / H_zz);
		double teta_ref = 0;
		//double phi_ref = std::atan2(-H_yy / sqrt(H_xx*H_xx + H_zz*H_zz));
		double phi_ref = 0;
		
		double u1 = m * g; //double u1 = sqrt(H_xx*H_xx + H_yy*H_yy + H_zz*H_zz);
		double u2 = I_xx * (-(a_phi+k_phi)*w_x_pred - a_phi*k_phi*(phi_pred-phi_ref));
		double u3 = I_yy * (-(a_teta+k_teta)*w_y_pred - a_teta*k_teta*(teta_pred-teta_ref));
		/// ALTERNATIVE
		
		double _u1 = m * g;
		double _u2 = I_xx * (-(a_phi+k_phi)*_w_x_pred - a_phi*k_phi*_phi_pred);
		double _u3 = I_yy * (-(a_teta+k_teta)*_w_y_pred - a_teta*k_teta*_teta_pred); 
		
		
		//ROS_INFO("remote_u1=%f u1 = %f", remote_u1, u1);
		
		/// Debug
		//u3 = glob_angleVel_msg.angular_velocity.y > 6. ? 6. : 0;
		//if (u3 > 5)
		//	ROS_INFO("msg: %f u3: %f\n", glob_angleVel_msg.angular_velocity.y, u3); 
		
		u2_kalman = u2;
		u3_kalman = u3;
		///ALTERNATIVE		
		_u2_kalman = _u2;
		_u3_kalman = _u3; 
		
		
		if (N != 0)
		{
			u2_kalman_q.pop_front();
			u3_kalman_q.pop_front();
			u2_kalman_q.push_back(u2);
			u3_kalman_q.push_back(u3);

			w_x_pred_q.pop_front();
			w_y_pred_q.pop_front();
			w_x_pred_q.push_back(w_x_pred);
			w_y_pred_q.push_back(w_y_pred);
		}
		
		///ALTERNATIVE
		if (_N != 0)
		{
			_u2_kalman_q.pop_front();
			_u2_kalman_q.push_back(_u2);
			_u3_kalman_q.pop_front();
			_u3_kalman_q.push_back(_u3);
		}
	
		geometry_msgs::PointStamped msg;
		msg.header.stamp = ros::Time::now();
		msg.point.x = u1;
		msg.point.y = u2;
		msg.point.z = u3;
		///ALTERNATIVE
		geometry_msgs::PointStamped _msg;
		_msg.header.stamp = ros::Time::now();
		_msg.point.x = _u1;
		_msg.point.y = _u2;
		_msg.point.z = _u3;	
		
		sensor_msgs::Imu msg_mod = glob_angleVel_msg;
		msg_mod.header.stamp = ros::Time::now();
		msg_mod.angular_velocity.x = w_x_pred;
		msg_mod.angular_velocity.y = w_y_pred;
		///ALTERNATIVE
		sensor_msgs::Imu _msg_mod;
		_msg_mod.header.stamp = ros::Time::now();
		_msg_mod.angular_velocity.x = _w_x_pred;
		_msg_mod.angular_velocity.y = _w_y_pred;
		
		geometry_msgs::Vector3Stamped msg_rpy_pred;
		msg_rpy_pred.header.stamp = ros::Time::now();
		msg_rpy_pred.vector.x = phi_pred;
		msg_rpy_pred.vector.y = teta_pred;
		msg_rpy_pred.vector.z = teta_measure;
		///ALTERNATIVE
		geometry_msgs::Vector3Stamped _msg_rpy_pred;
		_msg_rpy_pred.header.stamp = ros::Time::now();
		_msg_rpy_pred.vector.x = _phi_pred;
		_msg_rpy_pred.vector.y = _teta_pred;
		_msg_rpy_pred.vector.z = 0;

	
		chatter_pub.publish(msg);
		_chatter_pub.publish(_msg);
		
		chatter_imu_prefilter.publish(msg_pf);
		chatter_rpy_prefilter.publish(msg_rpy_pf);
		
		chatter_imu_stepped.publish(msg_st);
		chatter_rpy_stepped.publish(msg_rpy_stepped);
		
		chatter_pub_mod.publish(msg_mod);
		chatter_rpy_pred.publish(msg_rpy_pred);
		_chatter_pub_mod.publish(_msg_mod);
		_chatter_rpy_pred.publish(_msg_rpy_pred);
		*/
		loop_rate.sleep();
	}

	dr_file.close();
	dpf_file.close();
	_dm_file.close();
	_u_file.close();

	return 0;
}
