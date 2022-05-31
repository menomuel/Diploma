#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>

#include <vector>
#include <queue>
#include <deque>
#include <sstream>
#include <random>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <fstream>


geometry_msgs::Vector3Stamped glob_rpy_msg; 
sensor_msgs::Imu glob_angleVel_msg;

geometry_msgs::Vector3Stamped glob_camXYZ_msg;
geometry_msgs::Vector3Stamped glob_camVel_msg;
geometry_msgs::Vector3Stamped glob_camRPY_msg;
geometry_msgs::QuaternionStamped glob_camRef_msg;

geometry_msgs::PointStamped glob_remote_msg;

bool imuUp = false;
bool camXYZUp = false; // separate for XYZ and Vel??
bool camVelUp = false; // separate for XYZ and Vel??
bool camRPYUp = false;
bool camRefUp = false;


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

void remoteCallback(const geometry_msgs::PointStamped& msg)
{
	glob_remote_msg = msg;
	//ROS_INFO("Remote caught");
}


void camXYZCallback(const geometry_msgs::Vector3Stamped& msg)
{
	glob_camXYZ_msg = msg;
	if (!camXYZUp)
		camXYZUp = true;
}

void camVelCallback(const geometry_msgs::Vector3Stamped& msg)
{
	glob_camVel_msg = msg;
	if (!camVelUp)
		camVelUp = true;
}

void camRPYCallback(const geometry_msgs::Vector3Stamped& msg)
{
	glob_camRPY_msg = msg;
	if (!camRPYUp)
		camRPYUp = true;
}

void camRefCallback(const geometry_msgs::QuaternionStamped& msg)
{
	glob_camRef_msg = msg;
	camRefUp = true;
}

int main(int argc, char **argv)
{	
	
	ros::init(argc, argv, "controller");
	ros::NodeHandle n("~");
	
	int _param_N;
	double param_rw, param_ra;
	double param_qw, param_qa;

	n.param<int>("N", _param_N, 4);
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
	std::ofstream _dst_file;
	std::ofstream _u_file;
	std::ofstream cam_file;
	dr_file.open("dr_13_04_model_phi1e-1.csv");
	dpf_file.open("dpf_13_04_model_phi1e-1.csv");
	_dst_file.open("_dst_13_04_model_phi1e-1.csv");
	_dm_file.open("_dm_13_04_model_phi1e-1.csv");
	_u_file.open("_u_13_04_model_phi1e-1.csv");
	cam_file.open("cam_13_04_model_phi1e-1.csv");
	
	std::ofstream rpyf_file;
	rpyf_file.open("rpyf_13_04_model_phi1e-1.csv");
	
	ros::Subscriber subRPY = n.subscribe("/imu/rpy/filtered", 1, rpyCallback);
	ros::Subscriber subAngleVel = n.subscribe("/imu/data_raw", 1, angleVelCallback); //with filter subscribe on /imu/data!
	ros::Subscriber subRemote = n.subscribe("/remote", 1, remoteCallback);
	ros::Subscriber subCamXYZ = n.subscribe("/camera/xyz/kalman", 1, camXYZCallback);
	ros::Subscriber subCamVel = n.subscribe("/camera/vel/kalman", 1, camVelCallback);
	ros::Subscriber subCamRPY = n.subscribe("/camera/rpy/kalman", 1, camRPYCallback);
	//ros::Subscriber subCamXYZ = n.subscribe("/camera/xyz/2degree", 1, camXYZCallback);
	//ros::Subscriber subCamVel = n.subscribe("/camera/vel/2degree", 1, camVelCallback);
	ros::Subscriber subCamRef = n.subscribe("/camera/ref", 1, camRefCallback);

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::QuaternionStamped>("/controls", 1);
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

	//double m = 0.4; // [kg]
	//double g = 9.81; // [m/c^2]

	double I_xx = 0.0075; // 0.0075; //0.0464; // [kg*m^2] or 0.01
	double I_yy = 0.0075; // 0.0075; // 0.0075; //0.0464; // [kg*m^2] or 0.01
	double I_zz = 0.0075; // 0.0075;

	double k_x, a_x, k_y, a_y, k_z, a_z;
	k_x = a_x = k_y = a_y = k_z = a_z = 4.0;

	double k_teta, a_teta, k_phi, a_phi, k_psi, a_psi;
	k_teta = a_teta = k_phi = a_phi = k_psi = a_psi = 10; // flight - 5.0, model - 16.0; 
	
	double dt = 1./200;
	double dt_fwd = 1./200; 
	int _N = _param_N;
	ROS_INFO("Steps equal to %d", _N);
	
	// double t_prev = ros::Time::now().toSec();
	double t_prev, t_ref;
	double T = 0.15; // [s], filter param
	double k_f = 1. / T;
	
	// PYTHON
	double w_x = 0.;
	double phi = 0.;
	double w_y = 0.;
	double teta = 0.;
	double w_z = 0.;
	double psi = 0.;
	
	double acc_filter_val_x = 0;
	double acc_filter_val_y = 0;
	double acc_filter_val_z = 0;
	double gyro_filter_val_x = 0;
	double gyro_filter_val_y = 0;
	double gyro_filter_val_z = 0;
	
	double w_x_measure, phi_measure;
	double w_y_measure, teta_measure;
	double w_z_measure, psi_measure;
	
	double _w_x_stepped, _w_x_pf_stepped, _phi_stepped, _phi_pf_stepped;
	double _w_y_stepped, _w_y_pf_stepped, _teta_stepped, _teta_pf_stepped;
	double _w_z_stepped, _w_z_pf_stepped, _psi_stepped;

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
			  0, 0, param_ra*10, 0,
			  0, 0, 0, param_ra/10*1;
	
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
	
	/// FOR YAW
	Eigen::Matrix3d _Fz;
	_Fz << 	       1,           0,        0,
				k_f*dt, 1-k_f*dt,        0,
				      dt,           0,        1;
	
	Eigen::Vector3d _Bz {dt / I_zz, 0, 0};
	
	Eigen::Matrix3d _Qz;
	_Qz << param_qw, 0, 0, 
			  0, param_qw/10, 0,
			  0, 0, param_qa;
	
	Eigen::Matrix3d _Rz;
	_Rz << param_rw, 0, 0, 
			  0, param_rw/10, 0,
			  0, 0, param_ra;

	Eigen::Matrix3d _Pz;
	_Pz << _p, 0, 0,
			  0, _p, 0,
			  0, 0, _p;
			  
	Eigen::Matrix3d _Hz = Eigen::MatrixXd::Identity(3, 3);

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
	
	std::deque<double> _u4_kalman_q(_N, 0);
	double _u4_kalman = 0;
	double _w_z_pred = 0;
	double _w_z_pf_pred = 0;
	double _psi_pred = 0;
		
	double _u1, _u2, _u3, _u4;

	int count = 0;
	int count_ref = 0;
	while (ros::ok())
	{
		ros::spinOnce();
		
		if (count == 0)
			t_prev = ros::Time::now().toSec();
		
		double dt_filter = ros::Time::now().toSec() - t_prev;
		t_prev = ros::Time::now().toSec();

		bool modelOn = true;
		
		if (imuUp && camRefUp) // && camXYZUp && camVelUp && camRPYUp) // WITH CAMERA
		{
			//if (count_ref == 0)
			//{
			//	t_ref = ros::Time::now().toSec();
			//	++count_ref;
			//}

			w_x = glob_angleVel_msg.angular_velocity.x;
			w_y = glob_angleVel_msg.angular_velocity.y;
			w_z = glob_angleVel_msg.angular_velocity.z;
			phi = glob_angleVel_msg.linear_acceleration.x;
			teta = glob_angleVel_msg.linear_acceleration.y;
			psi = glob_angleVel_msg.linear_acceleration.z;			

			/*
			if (!modelOn)
			{
				if (psi > 20)
					psi = 20;
				else if (psi < 4)
					psi = 4;
			}
			*/
			
			w_x_measure = w_x;
			w_y_measure = w_y;
			w_z_measure = w_z;
			phi_measure = phi;
			teta_measure = teta;
			psi_measure = psi;
			
			// PREFILTER
			acc_filter_val_x = acc_filter_val_x - 1/T * (acc_filter_val_x -  phi_measure) * dt_filter;
			gyro_filter_val_x = gyro_filter_val_x - 1/T * (gyro_filter_val_x -  w_x_measure) * dt_filter;
			acc_filter_val_y = acc_filter_val_y - 1/T * (acc_filter_val_y -  teta_measure) * dt_filter;
			gyro_filter_val_y = gyro_filter_val_y - 1/T * (gyro_filter_val_y -  w_y_measure) * dt_filter;
			acc_filter_val_z = acc_filter_val_z - 1/T * (acc_filter_val_z -  psi_measure) * dt_filter;
			gyro_filter_val_z = gyro_filter_val_z - 1/T * (gyro_filter_val_z -  w_z_measure) * dt_filter;

			double w_x_pf = gyro_filter_val_x;
			double w_y_pf = gyro_filter_val_y;
			double w_z_pf = gyro_filter_val_z;

			double phi_pf = 0, teta_pf = 0, psi_pf = 0;
			if (modelOn)
			{
				phi_pf = acc_filter_val_x;
				teta_pf = acc_filter_val_y;
				psi_pf = acc_filter_val_z;
			} else
			{				
				// ??????? atan2 valid ???????
				/*
				phi_measure = std::atan2(teta, psi);
				teta_measure = - std::atan2(phi, psi);
				phi_pf = std::atan2(acc_filter_val_y, acc_filter_val_z);
				teta_pf = - std::atan2(acc_filter_val_x, acc_filter_val_z);
				*/
				
				/// ALTERNATIVE
				phi_measure = std::atan2(teta, sqrt(phi*phi + psi*psi));
				teta_measure = - std::atan2(phi, sqrt(teta*teta + psi*psi));
				phi_pf = std::atan2(acc_filter_val_y, sqrt(acc_filter_val_x*acc_filter_val_x + acc_filter_val_z*acc_filter_val_z));
				teta_pf = - std::atan2(acc_filter_val_x, sqrt(acc_filter_val_y*acc_filter_val_y + acc_filter_val_z*acc_filter_val_z));
			}
			psi_measure = glob_camRPY_msg.vector.z; // from camera
			
						
			dr_file << glob_angleVel_msg.header.stamp << "," << w_x_measure << "," << w_y_measure << "," << w_z_measure << "," << phi_measure << "," << teta_measure << "," << psi_measure << ","
			<< phi << "," << teta << "," << psi << "\n";
			dpf_file << ros::Time::now() << "," << w_x_pf << "," << w_y_pf << "," << w_z_pf << "," << phi_pf << "," << teta_pf << "," << psi_pf << "\n";

			rpyf_file << glob_rpy_msg.header.stamp << "," <<  glob_rpy_msg.vector.x << "," << glob_rpy_msg.vector.y << "," << glob_rpy_msg.vector.z << "\n";
		
			// STEP
			_w_x_stepped = w_x_measure;
			_w_x_pf_stepped = w_x_pf;
			_phi_stepped = phi_measure;
			_phi_pf_stepped = phi_pf;
			
			_w_y_stepped = w_y_measure;
			_w_y_pf_stepped = w_y_pf;
			_teta_stepped = teta_measure;
			_teta_pf_stepped = teta_pf;
			
			_w_z_stepped = w_z_measure;
			_w_z_pf_stepped = w_z_pf;
			_psi_stepped = psi_measure; //psi from camera already stepped!

			for (int i = 0; i < _N; ++i)
			{
				_phi_stepped += _w_x_stepped * dt_fwd;
				_w_x_stepped += (_u2_kalman_q[i] / I_xx) * dt_fwd;
				_w_x_pf_stepped += - k_f * _w_x_pf_stepped * dt + k_f * _w_x_stepped * dt;
				_phi_pf_stepped += - k_f * _phi_pf_stepped * dt + k_f * _phi_stepped * dt;

				_teta_stepped += _w_y_stepped * dt_fwd;
				_w_y_stepped += (_u3_kalman_q[i] / I_yy) * dt_fwd;
				_w_y_pf_stepped += - k_f * _w_y_pf_stepped * dt + k_f * _w_y_stepped * dt;
				_teta_pf_stepped += - k_f * _teta_pf_stepped * dt + k_f * _teta_stepped * dt;

				_w_z_stepped += (_u4_kalman_q[i] / I_zz) * dt_fwd;
				_w_z_pf_stepped += - k_f * _w_z_pf_stepped * dt + k_f * _w_z_stepped * dt;
			}
			
			_dst_file << ros::Time::now() << "," << _w_x_stepped << "," << _w_y_stepped << "," << _w_z_stepped << "," << _phi_stepped << "," << _teta_stepped << "," << _psi_stepped
			 << "," << _w_x_pf_stepped << "," << _w_y_pf_stepped << "," << _phi_pf_stepped << "," << _teta_pf_stepped << "\n";
			
			// KALMAN FILTER-4
			/// x-axis
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
			
			/// z-axis
			Eigen::Vector3d _Xz_k_prev {_w_z_pred, _w_z_pf_pred, _psi_pred};
			Eigen::Vector3d _Xz_k = _Fz * _Xz_k_prev + _Bz * _u4_kalman;
			
			_Pz = _Fz * _Pz * _Fz.transpose() + _Qz;
			
			Eigen::Vector3d _Zz_k {_w_z_stepped, _w_z_pf_stepped, _psi_stepped};
			Eigen::Vector3d _Yz_k = _Zz_k - _Hz * _Xz_k;
			
			Eigen::Matrix3d _Sz_k = _Hz * _Pz * _Hz.transpose() + _Rz;
			Eigen::Matrix3d _Kz_k = _Pz * _Hz.transpose() * _Sz_k.inverse();
			
			_Xz_k += _Kz_k * _Yz_k;
			_Pz = (Eigen::MatrixXd::Identity(3, 3) - _Kz_k * _Hz) * _Pz;

			_w_z_pred = _Xz_k(0);
			_w_z_pf_pred = _Xz_k(1);
			_psi_pred = _Xz_k(2);
			
			_dm_file << ros::Time::now() << "," << _w_x_pred << "," << _w_y_pred << "," << _w_z_pred  << "," << _phi_pred << "," << _teta_pred << "," <<  _psi_pred
			<< "," << _w_x_pf_pred  << "," << _w_y_pf_pred << "," << _w_z_pf_pred << "\n";
			
			double phi_ref = 0, teta_ref = 0, psi_ref = 0;

			// Only for diploma plots
			//double timelim = 5;
			//if (ros::Time::now().toSec() - t_ref > timelim)
			//	phi_ref = 0.3;


			// CONTROLS
			//teta_ref = -0.5;
			phi_ref = glob_camRef_msg.quaternion.y;
			teta_ref = glob_camRef_msg.quaternion.z;
			psi_ref = glob_camRef_msg.quaternion.w;

			//double m = 0.43, g = 9.81;
			//_u1 = m * g; // no camera
			_u1 = glob_camRef_msg.quaternion.x;
			
			_u2 = I_xx * (-(a_phi+k_phi)*_w_x_pred - a_phi*k_phi*(_phi_pred-phi_ref));
			_u3 = I_yy * (-(a_teta+k_teta)*_w_y_pred - a_teta*k_teta*(_teta_pred-teta_ref));

			//_u2 = I_xx * (-(a_phi+k_phi)*w_x_measure - a_phi*k_phi*(phi_measure-phi_ref));
			//_u3 = I_yy * (-(a_teta+k_teta)*w_y_measure - a_teta*k_teta*(teta_measure-teta_ref));
			
			//_u2 = I_xx * (-(a_phi+k_phi)*w_x_pf - a_phi*k_phi*(phi_pf-phi_ref));
			//_u3 = I_yy * (-(a_teta+k_teta)*w_y_pf - a_teta*k_teta*(teta_pf-teta_ref));
			
			//_u2 = I_xx * (-(a_phi+k_phi)*_w_x_stepped - a_phi*k_phi*(_phi_stepped-phi_ref)); // ZERO replaced _w_x_pred
			//_u3 = I_yy * (-(a_teta+k_teta)*_w_y_stepped - a_teta*k_teta*(_teta_stepped-teta_ref)); // ZERO
			
			//_u4 = I_zz * (-(a_psi+k_psi)*_w_z_stepped - a_psi*k_psi*(_psi_stepped - psi_ref));
			_u4 = I_zz * (-(a_psi+k_psi)*_w_z_pred - a_psi*k_psi*(_psi_pred - psi_ref));
			
			cam_file << ros::Time::now() << "," << _u1 << "," << phi_ref << "," << teta_ref << "," << psi_ref << "\n";

			// LIMITS
			
			if (!modelOn)
			{
				_u1 = _u1 < glob_remote_msg.point.x ? _u1 : glob_remote_msg.point.x;
				_u2 += glob_remote_msg.point.y;
				_u3 += glob_remote_msg.point.z;
			}
			
			if (_u1 > 8)
				_u1 = 8;
			
			double ulim = 0.5; // model - 0.5, flight - 0.15;
			double unorm = sqrt(_u2*_u2+_u3*_u3);	
			if (unorm > ulim)
			{
				_u2 *= ulim / unorm;
				_u3 *= ulim / unorm;
			}
			
			if (_u4 > ulim)
				_u4 = ulim;
			else if (_u4 < -ulim)
				_u4 = -ulim;
			
			_u2_kalman = _u2;
			_u3_kalman = _u3;
			_u4_kalman = _u4;

			_u_file << ros::Time::now() << "," << _u1 << "," << _u2 << "," << _u3 << "," << _u4 << "\n";
			

			if (_N != 0)
			{
				_u2_kalman_q.pop_front();
				_u2_kalman_q.push_back(_u2);
				_u3_kalman_q.pop_front();
				_u3_kalman_q.push_back(_u3);
				_u4_kalman_q.pop_front();
				_u4_kalman_q.push_back(_u4);
			}
			
			// MESSAGES			
			geometry_msgs::QuaternionStamped msg;
			msg.header.stamp = ros::Time::now();
			msg.quaternion.x = _u1;
			msg.quaternion.y = _u2;
			msg.quaternion.z = _u3;
			msg.quaternion.w = _u4;
			
			sensor_msgs::Imu msg_pf;
			msg_pf.header.stamp = ros::Time::now();
			msg_pf.linear_acceleration.x = acc_filter_val_x;
			msg_pf.angular_velocity.x = gyro_filter_val_x;
			msg_pf.linear_acceleration.y = acc_filter_val_y;
			msg_pf.angular_velocity.y = gyro_filter_val_y;
			msg_pf.linear_acceleration.z = acc_filter_val_z;
			msg_pf.angular_velocity.z = gyro_filter_val_z;
			
			sensor_msgs::Imu _msg_st;
			_msg_st.header.stamp = ros::Time::now();
			_msg_st.angular_velocity.x = _w_x_stepped;
			_msg_st.angular_velocity.y = _w_y_stepped;
			_msg_st.angular_velocity.z = _w_z_stepped;
			
			geometry_msgs::Vector3Stamped _msg_rpy_stepped;
			_msg_rpy_stepped.header.stamp = ros::Time::now();
			_msg_rpy_stepped.vector.x = _phi_stepped;
			_msg_rpy_stepped.vector.y = _teta_stepped;
			_msg_rpy_stepped.vector.z = _psi_stepped;
			
			sensor_msgs::Imu _msg_mod;
			_msg_mod.header.stamp = ros::Time::now();
			_msg_mod.angular_velocity.x = _w_x_pred;
			_msg_mod.angular_velocity.y = _w_y_pred;
			_msg_mod.angular_velocity.z = _w_z_stepped; // stepped!
			
			geometry_msgs::Vector3Stamped _msg_rpy_pred;
			_msg_rpy_pred.header.stamp = ros::Time::now();
			_msg_rpy_pred.vector.x = _phi_pred;
			_msg_rpy_pred.vector.y = _teta_pred;
			_msg_rpy_pred.vector.z = _psi_stepped; // stepped
			
			// PUBLISHERS			
			chatter_pub.publish(msg);
			chatter_imu_prefilter.publish(msg_pf);
			_chatter_imu_stepped.publish(_msg_st);
			_chatter_rpy_stepped.publish(_msg_rpy_stepped);
		
			_chatter_pub_mod.publish(_msg_mod);
			_chatter_rpy_pred.publish(_msg_rpy_pred);
			
			++count;
		}

		loop_rate.sleep();
	}

	dr_file.close();
	dpf_file.close();
	_dm_file.close();
	_u_file.close();
	cam_file.close();
	
	rpyf_file.close();

	return 0;
}
