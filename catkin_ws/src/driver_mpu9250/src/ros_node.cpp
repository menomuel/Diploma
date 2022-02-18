#include "ros_node.h"

#include <sensor_msgs_ext/accelerometer.h>
#include <sensor_msgs_ext/gyroscope.h>
#include <sensor_msgs_ext/magnetometer.h>
#include <sensor_msgs_ext/temperature.h>

// TODO: CUSTOM MARKS
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <vector>

#include <cmath>

// TIME
#include <chrono>

// COM PORT
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */


static int fd, id, fd1; /* File descriptor for the port */
static ros::Time startTime, currTime;

// CONSTRUCTORS
ros_node::ros_node(std::shared_ptr<driver> driver, int argc, char **argv)
{
	ros::Time::init();
	startTime = ros::Time::now();
	id = 0;
	
	fd = open("/dev/ttyS0", O_WRONLY);
	if (fd == -1)
	{
		fprintf(stderr, "open_port: Unable to open /dev/ttyS0");
	} else
	{
		fcntl(fd, F_SETFL, 0);
	}
	
    // Initialize flags.
    ros_node::f_gyroscope_calibrating = false;

    // Create a new driver.
    ros_node::m_driver = driver;

    // Initialize the ROS node.
    ros::init(argc, argv, "driver_mpu9250");

    // Get the node's handle.
    ros_node::m_node = std::make_shared<ros::NodeHandle>();

    // Read parameters.
    ros::NodeHandle private_node("~");
    int param_i2c_bus = private_node.param<int>("i2c_bus", 1);
    int param_i2c_address = private_node.param<int>("i2c_address", 0x68);
    int param_interrupt_pin = private_node.param<int>("interrupt_gpio_pin", 0);
    int param_gyro_dlpf_frequency = private_node.param<int>("gyro_dlpf_frequency", 0);
    int param_accel_dlpf_frequency = private_node.param<int>("accel_dlpf_frequency", 0);
    int param_gyro_fsr = private_node.param<int>("gyro_fsr", 0);
    int param_accel_fsr = private_node.param<int>("accel_fsr", 0);
    float param_max_data_rate = private_node.param<float>("max_data_rate", 8000.0F);

    // TODO: calib params
    //std::vector<double> param_accel_calib;
    //double sum = 0;
  //  private_node.getParam("calibration_accelerometer", param_accel_calib);
   //double sum = param_accel_calib[0]+ param_accel_calib[1];
   // for(unsigned i=0; i < param_accel_calib.size(); i++)
    //{
//	  sum += param_accel_calib[i];
  //  }
    //sum = param_accel_calib[0];

//    ROS_INFO_STREAM("sum:" << sum);

    std::vector<std::string> keys;
    private_node.getParamNames(keys);
    for(auto& key: keys)
	ROS_INFO_STREAM("Param: "<< key);
    

    // Read calibrations.
    ros_node::m_calibration_accelerometer.load(private_node, "/driver_mpu9250/calibration/accelerometer");
    ros_node::m_calibration_magnetometer.load(private_node, "calibration/magnetometer");

    // Set up data publishers.
    ros_node::m_publisher_accelerometer = ros_node::m_node->advertise<sensor_msgs_ext::accelerometer>("imu/accelerometer", 1);
    ros_node::m_publisher_gyroscope = ros_node::m_node->advertise<sensor_msgs_ext::gyroscope>("imu/gyroscope", 1);
    ros_node::m_publisher_magnetometer = ros_node::m_node->advertise<sensor_msgs_ext::magnetometer>("imu/magnetometer", 1);
    ros_node::m_publisher_temperature = ros_node::m_node->advertise<sensor_msgs_ext::temperature>("imu/temperature", 1);
    //TODO: CUSTOM ADVERTISE
    ros_node::m_publisher_imu_data = ros_node::m_node->advertise<sensor_msgs::Imu>("imu/data_raw", 1);
    //ros_node::m_publisher_imu_data_unfilter = ros_node::m_node->advertise<sensor_msgs::Imu>("imu/data_raw_unfilter", 1);

    // Initialize the driver and set parameters.
    try
    {
        // Attach the data callback.
        ros_node::m_driver->set_data_callback(std::bind(&ros_node::data_callback, this, std::placeholders::_1));
        // Initialize driver.
        ros_node::m_driver->initialize(static_cast<unsigned int>(param_i2c_bus), static_cast<unsigned int>(param_i2c_address), static_cast<unsigned int>(param_interrupt_pin));
        // Set parameters.
        ///ROS_INFO_STREAM("param_gyro_dlpf_frequency: " << param_gyro_dlpf_frequency << "\nparam_accel_dlpf_frequency: " << param_accel_dlpf_frequency << "\nparam_max_data_rate: " << param_max_data_rate << "\n");
        
        float data_rate = ros_node::m_driver->p_dlpf_frequencies(static_cast<driver::gyro_dlpf_frequency_type>(param_gyro_dlpf_frequency), static_cast<driver::accel_dlpf_frequency_type>(param_accel_dlpf_frequency), param_max_data_rate);
        ros_node::m_driver->p_gyro_fsr(static_cast<driver::gyro_fsr_type>(param_gyro_fsr));
        ros_node::m_driver->p_accel_fsr(static_cast<driver::accel_fsr_type>(param_accel_fsr));

        ROS_INFO_STREAM("mpu9250 driver successfully initialized on i2c bus " << param_i2c_bus << " at address 0x" << std::hex << param_i2c_address);
        ROS_INFO_STREAM("sensor data rate is " << data_rate << " hz");
    }
    catch (std::exception& e)
    {
        ROS_FATAL_STREAM(e.what());
        exit(1);
    }

    // Set up services.
    ros_node::m_service_calibrate_gyroscope = ros_node::m_node->advertiseService("imu/calibrate_gyroscope", &ros_node::service_calibrate_gyroscope, this);

    // Perform initial gyroscope calibration.
    ros_node::calibrate_gyroscope(500);
}

// ROS
void ros_node::spin()
{
    // Spin.
    ros::spin();

    // Deinitialize driver.
    ros_node::deinitialize_driver();
}

// SERVICES
bool ros_node::service_calibrate_gyroscope(sensor_msgs_ext::calibrate_gyroscopeRequest& request, sensor_msgs_ext::calibrate_gyroscopeResponse& response)
{
    response.success = ros_node::calibrate_gyroscope(request.averaging_period);
    if(!response.success)
    {
        response.message = "failed to collect enough points for calibration";
    }

    return true;
}
bool ros_node::calibrate_gyroscope(uint32_t averaging_period)
{
    // Convert averaging period to duration.
    ros::Duration averaging_duration(static_cast<double>(averaging_period) / 1000.0);

    // Clear the collection window.
    ros_node::m_gyroscope_calibration_window.clear();

    // Enable the data collection.
    ros_node::f_gyroscope_calibrating = true;

    // Sleep while gyro data is collected on interrupt thread.
    averaging_duration.sleep();

    // Disable the data collection.
    ros_node::f_gyroscope_calibrating = false;

    // Check if the window contains data.
    if(ros_node::m_gyroscope_calibration_window.size() < 5)
    {
        ROS_ERROR_STREAM("gyroscope calibration failed (not enough data: " << ros_node::m_gyroscope_calibration_window.size() << " points)");
        return false;
    }

    // Iterate through window to calculate average.
    Eigen::Vector3d average;
    average.setZero();
    for(auto point = ros_node::m_gyroscope_calibration_window.cbegin(); point != ros_node::m_gyroscope_calibration_window.cend(); ++point)
    {
        average += *point;
    }
    average /= static_cast<double>(ros_node::m_gyroscope_calibration_window.size());

    // Clear window.
    ros_node::m_gyroscope_calibration_window.clear();

    // Create new homogeneous transform by subtracting out bias.
    Eigen::Matrix4d calibration;
    calibration.setIdentity();
    calibration.block(0, 3, 3, 1) = -average;

    // Update gyroscope calibration.
    ros_node::m_calibration_gyroscope.update(calibration);

    // Log success.
    ROS_INFO_STREAM("gyroscope calibration completed with averaging period of " << averaging_period << " ms");

    return true;
}

// METHODS
void ros_node::deinitialize_driver()
{
    try
    {
        ros_node::m_driver->deinitialize();
        ROS_INFO_STREAM("driver successfully deinitialized");
    }
    catch (std::exception& e)
    {
        ROS_FATAL_STREAM(e.what());
    }
}


// CALLBACKS
void ros_node::data_callback(driver::data data)
{
   // TODO: CUSTOM MESSAGE
    sensor_msgs::Imu message_imu_data;
    //sensor_msgs::Imu message_imu_data_unfilter;

    message_imu_data.header.stamp = ros::Time::now();
    message_imu_data.header.frame_id = "raw_data";

    // Create accelerometer message.
    sensor_msgs_ext::accelerometer message_accel;
    // Set accelerations (convert from g's to m/s^2)
    message_accel.x = static_cast<double>(data.accel_x) * 9.80665;
    message_accel.y = static_cast<double>(data.accel_y) * 9.80665;
    message_accel.z = static_cast<double>(data.accel_z) * 9.80665;

    // Apply calibration.
    ros_node::m_calibration_accelerometer.calibrate(message_accel.x, message_accel.y, message_accel.z);
    // TODO: CUSTOM CALIBRATE ACCEL
    //ros_node::m_calibration_accelerometer.calibrate(message_imu_data.linear_acceleration.x,
    //						 message_imu_data.linear_acceleration.y,
    //						 message_imu_data.linear_acceleration.z);
    // TODO: CUSTOM SET ACCEL
    message_imu_data.linear_acceleration.x = message_accel.x;
    message_imu_data.linear_acceleration.y = message_accel.y;
    message_imu_data.linear_acceleration.z = message_accel.z;

    // Create gyroscope message.
    sensor_msgs_ext::gyroscope message_gyro;
    // Set rotation rates (convert from deg/sec to rad/sec)
    message_gyro.x = static_cast<double>(data.gyro_x) * M_PI / 180.0;
    message_gyro.y = static_cast<double>(data.gyro_y) * M_PI / 180.0;
    message_gyro.z = static_cast<double>(data.gyro_z) * M_PI / 180.0;


    // If gyroscope calibration is running, add uncalibrate data to window.
    if(ros_node::f_gyroscope_calibrating)
    {
        ros_node::m_gyroscope_calibration_window.push_back({message_gyro.x, message_gyro.y, message_gyro.z});
    }
    // Apply calibration.
    ros_node::m_calibration_gyroscope.calibrate(message_gyro.x, message_gyro.y, message_gyro.z);
    // TODO: CUSTOM CALIBRATE
    //ros_node::m_calibration_gyroscope.calibrate(message_imu_data.angular_velocity.x,
    //						message_imu_data.angular_velocity.y,
    //						message_imu_data.angular_velocity.z);

    // TODO: CUSTOM SET GYRO
    
    
    message_imu_data.angular_velocity.x = message_gyro.x;
    message_imu_data.angular_velocity.y = message_gyro.y;
    message_imu_data.angular_velocity.z = message_gyro.z;
    
    //message_imu_data_unfilter = message_imu_data;
    //message_imu_data_unfilter.angular_velocity.x = message_gyro.x;
    //message_imu_data_unfilter.angular_velocity.y = message_gyro.y;
    //message_imu_data_unfilter.angular_velocity.z = message_gyro.z;

    // Publish message.
    //ros_node::m_publisher_gyroscope.publish(message_gyro);

    // TODO: CUSTOM PUBLISH DATA
    //ros_node::m_publisher_imu_data.publish(message_imu_data);

    // Check if there was a magneto overflow.
    if(std::isnan(data.magneto_x) == false)
    {
        // Create magneto message.
        sensor_msgs_ext::magnetometer message_mag;
        // Fill magnetic field strengths (convert from uT to T)
        message_mag.x = static_cast<double>(data.magneto_x) * 0.000001;
        message_mag.y = static_cast<double>(data.magneto_y) * 0.000001;
        message_mag.z = static_cast<double>(data.magneto_z) * 0.000001;
        // Apply calibration.
        ros_node::m_calibration_magnetometer.calibrate(message_mag.x, message_mag.y, message_mag.z);
        // Publish message.
        //ros_node::m_publisher_magnetometer.publish(message_mag);
    }

    // Create temperature message.
    sensor_msgs_ext::temperature message_temp;
    message_temp.temperature = static_cast<double>(data.temp);
    // Publish temperature message.
    //ros_node::m_publisher_temperature.publish(message_temp);
	
	/*
	currTime = ros::Time::now();	
	if((currTime-startTime).toSec() >= 0.02)
	{
		startTime = currTime;
		
		ssize_t stat = write(fd, "A", 1);
		if (stat < 0)
			fprintf(stderr, "Unable to write to Serial port %d\n", fd);
	
		message_imu_data.angular_velocity.x = id;
		message_imu_data.angular_velocity.y = 5.;
		//ROS_INFO_STREAM("id: " << id << " time: " << ros::Time::now());  
		
		//t1 += std::chrono::milliseconds(10);
		id += 1;
		//std::cout << "diff: " << diff << std::endl;
	}
	*/
	
	
	// TODO: CUSTOM PUBLISH DATA
	//ros_node::m_publisher_imu_data_unfilter.publish(message_imu_data_unfilter);
    ros_node::m_publisher_imu_data.publish(message_imu_data);
}
