#include <ros/ros.h>

#include <std_msgs/Int64.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>

#include <cmath>
#include <signal.h>
#include <deque>
#include <random>

class Model {
    
    private:
		int counter;
		ros::Publisher pubImu;
		ros::Publisher pubRef;
		ros::Publisher pubXYZ;
		ros::Publisher pubVel;
		ros::Publisher pubRPY;
		ros::Publisher pubPose;

		ros::Subscriber subControls;
		
		double x, y, z;
		double vx, vy, vz;
		double phi, teta, psi;
		
		double mu_x, mu_y;
		double mass;
		
		double a_x, a_y, a_z; // accelerations
		double dot_x, dot_y, dot_z;
		double dot_phi, dot_teta, dot_psi;
	
		double torque_ref, phi_ref, teta_ref, psi_ref;

		double t_prev;
		double u[4];

		bool controlsUp = false;
		geometry_msgs::QuaternionStamped glob_msg_control;

		// Delay
		int _delay_N;
	    std::deque<double> _phi_delay_q;
    	std::deque<double> _w_x_delay_q;
		std::deque<double> _teta_delay_q;
    	std::deque<double> _w_y_delay_q;
    	std::deque<double> _psi_delay_q;
    	std::deque<double> _w_z_delay_q;

		// Noise
		double sigma_dot_angle;
		double sigma_angle;

		// IMU output
		double dot_phi_m, dot_teta_m, dot_psi_m;
		double phi_m, teta_m, psi_m;

    public:
		static constexpr double M = 0.43; // [kg]
		static constexpr double G = 9.81; // [m/s^2]
	
		static constexpr double I_xx = 0.0075; // 0.02;
		static constexpr double I_yy = 0.0075;
		static constexpr double I_zz = 0.0075;
    
    public:
		Model(double _x=0, double _y=0, double _z=0., double _phi=0., double _teta=0., double _psi=0., double weight=M, double mu = 0) :
			x(_x), y(_y), z(_z), vx(0), vy(0), vz(0), phi(_phi), teta(_teta), psi(_psi), mu_x(mu), mu_y(mu), mass(weight),
			dot_x(0), dot_y(0), dot_z(0), dot_phi(0), dot_teta(0), dot_psi(0)
		{
			t_prev = ros::Time::now().toSec();
			u[0] = u[1] = u[2] = u[3] = 0;

			/*
			_delay_N = 15;
			_phi_delay_q = std::deque<double>(_delay_N, 0);
			_w_x_delay_q = std::deque<double>(_delay_N, 0);
			_teta_delay_q = std::deque<double>(_delay_N, 0);
			_w_y_delay_q = std::deque<double>(_delay_N, 0);
			_psi_delay_q = std::deque<double>(_delay_N, 0);
			_w_z_delay_q = std::deque<double>(_delay_N, 0);
			*/
		}
		
		
		void set_handler(ros::NodeHandle *nh) {
			counter = 0;
			
			nh->param<int>("delay_N", _delay_N, 0);
			nh->param<double>("sigmW", sigma_dot_angle, 0);
			nh->param<double>("sigmA", sigma_angle, 0);
			ROS_INFO("delay_N = %d", _delay_N);
			ROS_INFO("noise sigma (ang. vel) = %lf", sigma_dot_angle);
			ROS_INFO("noise sigma (ang.) = %lf", sigma_angle);
			
			
			_phi_delay_q = std::deque<double>(_delay_N, 0);
			_w_x_delay_q = std::deque<double>(_delay_N, 0);
			_teta_delay_q = std::deque<double>(_delay_N, 0);
			_w_y_delay_q = std::deque<double>(_delay_N, 0);
			_psi_delay_q = std::deque<double>(_delay_N, 0);
			_w_z_delay_q = std::deque<double>(_delay_N, 0);
			
			
			subControls = nh->subscribe("/controls", 1,	&Model::callback_controls, this);
			
			pubImu = nh->advertise<sensor_msgs::Imu>("/imu/data_raw", 1);
			pubRef = nh->advertise<geometry_msgs::QuaternionStamped>("/camera/ref", 1);
			pubXYZ = nh->advertise<geometry_msgs::Vector3Stamped>("/camera/xyz/kalman", 1);
			pubVel = nh->advertise<geometry_msgs::Vector3Stamped>("/camera/vel/kalman", 1);
			pubRPY = nh->advertise<geometry_msgs::Vector3Stamped>("/camera/rpy/kalman", 1);
			pubPose = nh->advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
		}
		
		void set_control(const double *_u) {
			u[0] = _u[0];
			u[1] = _u[1];
			u[2] = _u[2];
			u[3] = _u[3];
		}
		
		void step(double dt) {
			if (controlsUp)
			{
				phi += dot_phi * dt;
				teta += dot_teta * dt;
				psi += dot_psi * dt;
				
				dot_phi += (u[1] - (I_zz - I_yy) * dot_teta * dot_psi) / I_xx * dt;
				dot_teta += (u[2] - (I_xx - I_zz) * dot_phi * dot_psi) / I_yy * dt;
				dot_psi += u[3] / I_zz * dt;
				
				a_x = (sin(psi) * sin(phi) + cos(psi) * cos(phi) * sin(teta)) * u[0] / mass;
				dot_x += ((sin(psi) * sin(phi) + cos(psi) * cos(phi) * sin(teta)) * u[0] / mass) * dt;
				x += dot_x * dt;

				a_y = (- cos(psi) * sin(phi) + sin(psi) * cos(phi) * sin(teta)) * u[0] / mass;
				dot_y += ((- cos(psi) * sin(phi) + sin(psi) * cos(phi) * sin(teta)) * u[0] / mass) * dt;
				y += dot_y * dt;

				double R = 0;
				if (z < 0.22)
					R = mass * G;
				a_z = (cos(phi) * cos(teta) * u[0] - mass * G + R) / mass;
				dot_z += ((cos(phi) * cos(teta) * u[0] - mass * G + R) / mass) * dt;
				z += dot_z * dt;

				// Calculate references
				/*
				double m = M; // [kg]
				double g = G; // [m/s^2]

				double x_ref = -0.3, y_ref = -0.3, z_ref = 1.;
				double a_x, k_x, a_y, k_y, a_z, k_z;
				a_x = k_x = a_y = k_y = a_z = k_z = 4.;

				double H_xx = - (a_x+k_x)*dot_x - a_x*k_x*(x-x_ref);
				double H_yy = - (a_y+k_y)*dot_y - a_y*k_y*(y-y_ref);
            	double H_zz = - (a_z+k_z)*dot_z - a_z*k_z*(z-z_ref) + g;

				torque_ref = m * sqrt(H_xx*H_xx + H_yy*H_yy + H_zz*H_zz);
                teta_ref = std::atan2(H_xx, H_zz);
                phi_ref = std::atan2(-H_yy, sqrt(H_xx*H_xx + H_zz*H_zz));
				psi_ref = 0;
				*/
			}
		}
		
		void add_delay_imu() {
			if (_delay_N != 0)
			{
				phi_m = _phi_delay_q.front();
				dot_phi_m = _w_x_delay_q.front();
				_phi_delay_q.pop_front();
				_w_x_delay_q.pop_front();
				_phi_delay_q.push_back(phi);
				_w_x_delay_q.push_back(dot_phi);

				teta_m = _teta_delay_q.front();
				dot_teta_m = _w_y_delay_q.front();
				_teta_delay_q.pop_front();
				_w_y_delay_q.pop_front();
				_teta_delay_q.push_back(teta);
				_w_y_delay_q.push_back(dot_teta);

				psi_m = _psi_delay_q.front();
				dot_psi_m = _w_z_delay_q.front();
				_psi_delay_q.pop_front();
				_w_z_delay_q.pop_front();
				_psi_delay_q.push_back(psi);
				_w_z_delay_q.push_back(dot_psi);
			}
		}
		
		void add_noise_imu(double sigma_dot_angle, double sigma_angle) {
			std::random_device rd{};
            std::mt19937 gen{rd()};
			std::normal_distribution<> d_w{0, sigma_dot_angle};
            std::normal_distribution<> d_a{0, sigma_angle};
            dot_phi_m += d_w(gen);
            dot_teta_m += d_w(gen);
            dot_psi_m += d_w(gen);
            phi_m += d_a(gen);
            teta_m += d_a(gen);
            psi_m += d_a(gen);
		}

		void callback_timer(const ros::TimerEvent& event) {
			double dt = ros::Time::now().toSec() - t_prev;
			t_prev = ros::Time::now().toSec();
			//ROS_INFO("dt %f", dt);

			double _u[4] {glob_msg_control.quaternion.x, glob_msg_control.quaternion.y,
					glob_msg_control.quaternion.z, glob_msg_control.quaternion.w};
			set_control(_u);
			step(dt);

			// Data disturbances
			dot_phi_m = dot_phi, dot_teta_m = dot_teta, dot_psi_m = dot_psi;
			phi_m = phi, teta_m = teta, psi_m = psi;

			add_delay_imu();
			add_noise_imu(sigma_dot_angle, sigma_angle);


			//ROS_INFO("Controls (%f, %f, %f, %f)", u[0], u[1], u[2], u[3]);
			tf::Quaternion quat;
			quat.setRPY(phi, teta, psi);
			quat = quat.normalize();

			// IMU MESSAGE
			sensor_msgs::Imu msg_imu;
			msg_imu.header.stamp = ros::Time::now();
			msg_imu.angular_velocity.x = dot_phi_m;
			msg_imu.angular_velocity.y = dot_teta_m;
			msg_imu.angular_velocity.z = dot_psi_m;
			msg_imu.linear_acceleration.x = phi_m;
			msg_imu.linear_acceleration.y = teta_m; // Shift!
			msg_imu.linear_acceleration.z = psi_m;
			pubImu.publish(msg_imu);

			++counter;
			 if (counter == 15) // 300/15=20hz //if (true)
			{
				counter = 0;

				// CAMERA MESSAGE
				geometry_msgs::QuaternionStamped msg_ref;
				msg_ref.header.stamp = ros::Time::now();
				msg_ref.quaternion.x = torque_ref;
				msg_ref.quaternion.y = phi_ref;
				msg_ref.quaternion.z = teta_ref;
				msg_ref.quaternion.w = psi_ref;

				geometry_msgs::PoseStamped msg_pose;
				msg_pose.header.stamp = ros::Time::now();
				msg_pose.pose.position.x = x;
				msg_pose.pose.position.y = y;
				msg_pose.pose.position.z = z;
				msg_pose.pose.orientation.x = quat.getX();
				msg_pose.pose.orientation.y = quat.getY();
				msg_pose.pose.orientation.z = quat.getZ();
				msg_pose.pose.orientation.w = quat.getW();
				//tf::Matrix3x3 m(quat);
				//double roll, pitch, yaw;
				//m.getRPY(roll, pitch, yaw);
				//ROS_INFO("%f %f %f %f angles (%f,%f,%f)", quat.getX(), quat.getY(), quat.getZ(), quat.getW(), roll, pitch, yaw);

				geometry_msgs::Vector3Stamped msg_xyz;
				msg_xyz.header.stamp = ros::Time::now();
				msg_xyz.vector.x = x;
				msg_xyz.vector.y = y;
				msg_xyz.vector.z = z;

				geometry_msgs::Vector3Stamped msg_vel;
				msg_vel.header.stamp = ros::Time::now();
				msg_vel.vector.x = dot_x;
				msg_vel.vector.y = dot_y;
				msg_vel.vector.z = dot_z;

				geometry_msgs::Vector3Stamped msg_rpy;
				msg_rpy.header.stamp = ros::Time::now();
				msg_rpy.vector.x = phi;
				msg_rpy.vector.y = teta;
				msg_rpy.vector.z = psi;

				//pubRef.publish(msg_ref);
				//pubXYZ.publish(msg_xyz);
				//pubVel.publish(msg_vel);
				//pubRPY.publish(msg_rpy);
				pubPose.publish(msg_pose);
			}
		}
		
		void callback_controls(const geometry_msgs::QuaternionStamped& msg) {
			glob_msg_control = msg;
			controlsUp = true;
		}
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "model_2013");
    ros::NodeHandle nh("~");
    
    Model model = Model();
    model.set_handler(&nh);
    
    ROS_INFO("Model initialized");
    
    int hz = 300;
    ros::Timer timer = nh.createTimer(ros::Duration(1./hz), &Model::callback_timer, &model);
    ros::spin();
}
