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

class Model {
    
    private:
		int counter;
		//ros::Publisher pub;
		ros::Publisher pubImu;
		ros::Publisher pubRef;
		ros::Publisher pubXYZ;
		ros::Publisher pubVel;
		ros::Publisher pubRPY;
		ros::Publisher pubPose;

		ros::Subscriber subControls;
		
		/*
		double m, g, mg;
		
		double tau_common, tau_real;
		double d_common, d_real;
		double k2_common, k2_real;
		double k_u0, k_phi, k_teta, k_psi; 
		
		
		m = 0.4; // [kg]
		g = 9.81 // [m/s^2]
		mg = m * g;
		
		tau_common = 0.17 //0.15
		tau_real = 0.17

		d_common = 0.35 //0.8
		d_real = 0.35

		k2_common = 1. //0.8
		k2_real = 1.

		k_u0 = 1.
		k_phi = 1.
		k_psi = 1.
		k_theta = 1.
		*/
			
		/*
		double _tau_common = 0.17; //0.15
		double _tau_real = 0.17;

		double _d_common = 0.35; //0.8
		double _d_real = 0.35;

		double _k2_common = 1.; //0.8
		double _k2_real = 1.;

		double _k_u0 = 1.;
		double _k_phi = 1.;
		double _k_psi = 1.;
		double _k_theta = 1.;
		*/
		double x, y, z;
		double vx, vy, vz;
		double phi, teta, psi;
		
		double mu_x, mu_y;
		double mass;
		
		double dot_x, dot_y, dot_z;
		double dot_phi, dot_teta, dot_psi;
	
		double torque_ref, phi_ref, teta_ref, psi_ref;

		double t_prev;
		double u[4];

		bool controlsUp = false;
		/*
		double phi_prev, teta_prev, psi_prev;
		double psi_ref, psi_ref_summ, psi_ref_prev;
		double F0x, F0y;
		double tau_phi, tau_teta, tau_psi;
		double d_phi, d_teta, d_psi;
		double k2_phi, k2_teta, k2_psi;
		double k_phi, k_teta, k_psi;
		double k1_phi, k1_teta, k1_psi;
		*/
		
		
	
		
		geometry_msgs::QuaternionStamped glob_msg_control;
    
    public:
		static constexpr double M = 0.365; // [kg]
		static constexpr double G = 9.81; // [m/s^2]
	
		static constexpr double I_xx = 0.02;
		static constexpr double I_yy = 0.02;
		static constexpr double I_zz = 0.02;
    
    public:
		Model(double _x=0, double _y=0, double _z=0., double _phi=0., double _teta=0., double _psi=0., double weight=M, double mu = 0) :
			x(_x), y(_y), z(_z), vx(0), vy(0), vz(0), phi(_phi), teta(_teta), psi(_psi), mu_x(mu), mu_y(mu), mass(weight),
			dot_x(0), dot_y(0), dot_z(0), dot_phi(0), dot_teta(0), dot_psi(0)
		{
			t_prev = ros::Time::now().toSec();
			u[0] = u[1] = u[2] = u[3] = 0;
		}
		
		
		void set_handler(ros::NodeHandle *nh) {
			counter = 0; 
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
				
				dot_x += ((sin(psi) * sin(phi) + cos(psi) * cos(phi) * sin(teta)) * u[0] / mass) * dt;
				x += dot_x * dt;

				dot_y += ((- cos(psi) * sin(phi) + sin(psi) * cos(phi) * sin(teta)) * u[0] / mass) * dt;
				y += dot_y * dt;
				

				double R = 0;
				if (z < 0.22)
					R = mass * G;
				dot_z += ((cos(phi) * cos(teta) * u[0] - mass * G + R) / mass) * dt;
				z += dot_z * dt;

				// Calculate references
				double m = 0.365; // [kg]
				double g = 9.81; // [m/s^2]

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

			}
		}
		
		void callback_timer(const ros::TimerEvent& event) {
			double dt = ros::Time::now().toSec() - t_prev;
			t_prev = ros::Time::now().toSec();
			
			//ROS_INFO("dt %f", dt);

			double _u[4] {glob_msg_control.quaternion.x, glob_msg_control.quaternion.y,
					glob_msg_control.quaternion.z, glob_msg_control.quaternion.w};
			set_control(_u);
			step(dt);


			//ROS_INFO("Controls (%f, %f, %f, %f)", u[0], u[1], u[2], u[3]);

			tf::Quaternion quat;
			quat.setRPY(phi, teta, psi);
			quat = quat.normalize();

			// IMU MESSAGE
			sensor_msgs::Imu msg_imu;
			msg_imu.header.stamp = ros::Time::now();
			msg_imu.angular_velocity.x = dot_phi;
			msg_imu.angular_velocity.y = dot_teta;
			msg_imu.angular_velocity.z = dot_psi;
			msg_imu.linear_acceleration.x = phi;
			msg_imu.linear_acceleration.y = teta;
			msg_imu.linear_acceleration.z = psi;
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
    ros::NodeHandle nh;
    
    Model model = Model();
    model.set_handler(&nh);
    
    ROS_INFO("Model initialized");
    
    int hz = 300;
    ros::Timer timer = nh.createTimer(ros::Duration(1./hz), &Model::callback_timer, &model);
    ros::spin();
}
