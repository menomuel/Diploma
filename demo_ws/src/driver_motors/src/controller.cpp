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

geometry_msgs::Vector3Stamped glob_camXYZ_msg;
geometry_msgs::Vector3Stamped glob_camVel_msg;

//double remote_u1 = 0;
//double remote_u2 = 0;
//double remote_u3 = 0;

bool imuUp = false;
bool camXYZUp = false; // separate for XYZ and Vel??
bool camVelUp = false; // separate for XYZ and Vel??

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
	std::ofstream _dst_file;
	std::ofstream _u_file;
	std::ofstream cam_file;
	dr_file.open("dr_13_04_model_phi1e-1.csv");
	dpf_file.open("dpf_13_04_model_phi1e-1.csv");
	_dst_file.open("_dst_13_04_model_phi1e-1.csv");
	_dm_file.open("_dm_13_04_model_phi1e-1.csv");
	_u_file.open("_u_13_04_model_phi1e-1.csv");
	cam_file.open("cam_13_04_model_phi1e-1.csv");
	
	ros::Subscriber subRPY = n.subscribe("/imu/rpy/filtered", 1, rpyCallback);
	ros::Subscriber subAngleVel = n.subscribe("/imu/data_raw", 1, angleVelCallback); //with filter subscribe on /imu/data!
	//ros::Subscriber subRemote = n.subscribe("/remote", 1, remoteCallback);
	ros::Subscriber subCamXYZ = n.subscribe("/camera/xyz", 1, camXYZCallback);
	ros::Subscriber subCamVel = n.subscribe("/camera/vel", 1, camVelCallback);

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

	double m = 0.4; // [kg]
	double g = 9.81; // [m/c^2]

	double I_xx = 0.02; //0.0464; // [kg*m^2] or 0.01
	double I_yy = 0.02; //0.0464; // [kg*m^2] or 0.01

	double k_x, a_x, k_y, a_y, k_z, a_z;
	k_x = a_x = k_y = a_y = k_z = a_z = 4.0;

	double k_teta, a_teta, k_phi, a_phi;
	k_teta = a_teta = k_phi = a_phi = 5.0;
	
	double dt = 1./200;
	double dt_fwd = 1./200; 
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
	double T = 0.02; // [s], filter param
	double k_f = 1. / T;
	
	double w_x_measure;
	double phi_measure;
	double w_y_measure;
	double teta_measure;
	double psi_measure;

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
			  0, param_rw, 0, 0,
			  0, 0, param_ra, 0,
			  0, 0, 0, param_ra;
	
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

	std::deque<double> x_delay_q(_delay_N, 0);
	std::deque<double> y_delay_q(_delay_N, 0);
	std::deque<double> z_delay_q(_delay_N, 0);
	std::deque<double> dx_delay_q(_delay_N, 0);
	std::deque<double> dy_delay_q(_delay_N, 0);
	std::deque<double> dz_delay_q(_delay_N, 0);

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

		bool modelOn = false;

		
		if (imuUp) //  && camXYZUp && camVelUp
		{
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
			if (!modelOn)
			{
				if (psi > 20)
					psi = 20;
				else if (psi < 4)
					psi = 4;
			}
			
			// AXES ALONG DIAGONAL
			/*
			const double pi = std::acos(-1);
			double w_x_rot = std::cos(pi/4) * w_x - std::sin(pi/4) * w_y;
			double w_y_rot = std::sin(pi/4) * w_x + std::cos(pi/4) * w_y;
			double phi_rot = std::cos(pi/4) * phi - std::sin(pi/4) * teta;
			double teta_rot = std::sin(pi/4) * phi + std::cos(pi/4) * teta;
			w_x = w_x_rot;
			w_y = w_y_rot;
			phi = phi_rot;
			teta = teta_rot;
			*/
			
			//dr_file << glob_angleVel_msg.header.stamp << "," << w_x << "," << w_y << "," << phi << "," << teta << "," << psi << "\n";

			// DELAY	
			/*
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
			*/

			// NO DELAY
			
			w_x_measure = w_x;
			w_y_measure = w_y;
			phi_measure = phi;
			teta_measure = teta;
			psi_measure = psi;
			

			// PREFILTER
			acc_filter_val_x = acc_filter_val_x - 1/T * (acc_filter_val_x -  phi_measure) * dt_filter;
			gyro_filter_val_x = gyro_filter_val_x - 1/T * (gyro_filter_val_x -  w_x_measure) * dt_filter;
			
			acc_filter_val_y = acc_filter_val_y - 1/T * (acc_filter_val_y -  teta_measure) * dt_filter;
			gyro_filter_val_y = gyro_filter_val_y - 1/T * (gyro_filter_val_y -  w_y_measure) * dt_filter;
			
			acc_filter_val_z = acc_filter_val_z - 1/T * (acc_filter_val_z -  psi_measure) * dt_filter;
			//dpf_file << ros::Time::now() << "," << gyro_filter_val_x << "," << gyro_filter_val_y << "," << acc_filter_val_x << "," << acc_filter_val_y << "," << acc_filter_val_z << "\n";
			//ROS_INFO("w_x_pf=%.8f phi_pf=%.8f", gyro_filter_val_x, acc_filter_val_x);

			double w_x_pf = gyro_filter_val_x;
			double w_y_pf = gyro_filter_val_y;

			double phi_pf = 0, teta_pf = 0, psi_pf = 0;
			if (modelOn)
			{
				phi_pf = acc_filter_val_x;
				teta_pf = acc_filter_val_y;
			} else
			{				
				phi_measure = std::atan2(teta, psi);
				teta_measure = - std::atan2(phi, psi);
				phi_pf = std::atan2(acc_filter_val_y, acc_filter_val_z);
				teta_pf = - std::atan2(acc_filter_val_x, acc_filter_val_z);
			}
			
			dr_file << glob_angleVel_msg.header.stamp << "," << w_x_measure << "," << w_y_measure << "," << phi_measure << "," << teta_measure << "," << psi_measure << "\n";
			//ROS_INFO_STREAM(glob_angleVel_msg.header.stamp << "," << w_x_measure << "," << w_y_measure << "," << phi_measure << "," << teta_measure << "," << psi_measure);
			dpf_file << ros::Time::now() << "," << w_x_pf << "," << w_y_pf << "," << phi_pf << "," << teta_pf << "," << psi_pf << "\n";

		
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
			
			_dst_file << ros::Time::now() << "," << _w_x_stepped << "," << _w_y_stepped << "," << _phi_stepped << "," << _teta_stepped << "\n";
			
			// KALMAN FILTER-4
			/// x-axis
			Eigen::Vector4d _Xx_k_prev {_w_x_pred, _w_x_pf_pred, _phi_pred, _phi_pf_pred};
			Eigen::Vector4d _Xx_k = _F * _Xx_k_prev + _Bx * _u2_kalman;
			//ROS_INFO_STREAM("_Xx_k_prev\n" << _Xx_k_prev << " \n_Xx_k\n" << _Xx_k << "\n");

			_Px = _F * _Px * _F.transpose() + _Q;
			//ROS_INFO_STREAM("_Px\n" << _Px  << "\n");

			Eigen::Vector4d _Zx_k {_w_x_stepped, _w_x_pf_stepped, _phi_stepped, _phi_pf_stepped}; // ZERO ANGLE
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
			_dm_file << ros::Time::now() << "," << _w_x_pred << "," << _w_y_pred << "," << _w_x_pf_pred  << "," << _w_y_pf_pred << "," << _phi_pred << "," << _teta_pred << "\n";
			
			// CAMERA
			double x_cam = 0, y_cam = 0, z_cam = 0;
			double dx_cam = 0, dy_cam = 0, dz_cam = 0;
			double x_ref = 0, y_ref = 0, z_ref = 1;

			double x_cam_measure = 0, y_cam_measure = 0, z_cam_measure = 0;
			double dx_cam_measure = 0, dy_cam_measure = 0, dz_cam_measure = 0;
			if (modelOn && camXYZUp && camVelUp)
			{
				// NO DELAY
				x_cam = x_cam_measure = glob_camXYZ_msg.vector.x;
				y_cam = y_cam_measure = glob_camXYZ_msg.vector.y;
				z_cam = z_cam_measure = glob_camXYZ_msg.vector.z;
				dx_cam = dx_cam_measure = glob_camVel_msg.vector.x;
				dy_cam = dy_cam_measure = glob_camVel_msg.vector.y;
				dz_cam = dz_cam_measure = glob_camVel_msg.vector.z;

				// DELAY
				x_cam = x_delay_q.front();
				dx_cam = dx_delay_q.front();
				x_delay_q.pop_front();
				dx_delay_q.pop_front();
				x_delay_q.push_back(x_cam_measure);
				dx_delay_q.push_back(dx_cam_measure);
				
				y_cam = y_delay_q.front();
				dy_cam = dy_delay_q.front();
				y_delay_q.pop_front();
				dy_delay_q.pop_front();
				y_delay_q.push_back(y_cam_measure);
				dy_delay_q.push_back(dy_cam_measure);
				
				z_cam = z_delay_q.front();
				dz_cam = dz_delay_q.front();
				z_delay_q.pop_front();
				dz_delay_q.pop_front();
				z_delay_q.push_back(z_cam_measure);
				dz_delay_q.push_back(dz_cam_measure);

			}

			double H_xx = - (a_x+k_x)*dx_cam - a_x*k_x*(x_cam-x_ref);
			double H_yy = - (a_y+k_y)*dy_cam - a_y*k_y*(y_cam-y_ref);
			double H_zz = - (a_z+k_z)*dz_cam - a_z*k_z*(z_cam-z_ref) + g;

			double phi_ref = 0, teta_ref = 0;
			if (modelOn && camXYZUp && camVelUp)
			{
				teta_ref = std::atan2(H_xx, H_zz);
				phi_ref = std::atan2(-H_yy, sqrt(H_xx*H_xx + H_zz*H_zz));
			}

			cam_file << glob_camXYZ_msg.header.stamp << "," << x_cam << "," << y_cam << "," << z_cam << "," 
					<< dx_cam << "," << dy_cam << "," << dz_cam << "\n";

			// CONTROLS
			phi_ref = teta_ref = 0; // no camera
			_u1 = m * g; // no camera
			//_u1 = m * sqrt(H_xx*H_xx + H_yy*H_yy + H_zz*H_zz); // with camera
			//_u2 = I_xx * (-(a_phi+k_phi)*0 - a_phi*k_phi*(0.5)); // ESC test
			_u2 = I_xx * (-(a_phi+k_phi)*_w_x_stepped - a_phi*k_phi*(_phi_stepped-phi_ref)); // ZERO replaced _w_x_pred
			_u3 = I_yy * (-(a_teta+k_teta)*_w_y_stepped - a_teta*k_teta*(_teta_stepped-teta_ref)); // ZERO
			
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

		loop_rate.sleep();
	}

	dr_file.close();
	dpf_file.close();
	_dm_file.close();
	_u_file.close();
	cam_file.close();

	return 0;
}
