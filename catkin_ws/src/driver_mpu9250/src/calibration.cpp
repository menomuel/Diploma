#include "calibration.h"

#include <ros/names.h>

// CONSTRUCTORS
calibration::calibration()
{
    // Initialize matrices/vectors.
    calibration::m_calibration.setIdentity();
    calibration::m_u.setZero();
    calibration::m_u(3) = 1.0;
    calibration::m_c.setZero();
}

// CONFIGURATION
void calibration::load(ros::NodeHandle& node_handle, std::string param_name)
{
    // Try reading the calibration parameter.
    if(param_name == std::string("calibration/magnetometer"))
    {
	ROS_INFO_STREAM("Do not hardcode magn " << param_name);
	return;
    }

    std::vector<double> components {0.997204, 0.000000, 0.000000, 3.578879,
				    0.000000, 0.997632, 0.000000, 2.834417,
				    0.000000, 0.000000, 0.993778, 2.559210,
				    0.000000, 0.000000, 0.000000, 1.000000};
/*
    std::vector<double> components {1.000000, 0.000000, 0.000000, 0.000000,
				    0.000000, 1.000000, 0.000000, 0.000000,
				    0.000000, 0.000000, 1.000000, 0.000000,
				    0.000000, 0.000000, 0.000000, 1.000000};
*/

    if(!node_handle.getParam(param_name, components))
    {
        // Param not found, quit.
        ROS_INFO_STREAM("param not found " << param_name);
        //return;
    }

    // Check validity of parameter.
    if(components.size() != 16)
    {
        ROS_ERROR_STREAM("invalid parameter for 4x4 calibration matrix: " << param_name);
        return;
    }

    // Read in row-step order.
    uint32_t k = 0;
    for(uint32_t i = 0; i < calibration::m_calibration.rows(); ++i)
    {
        for(uint32_t j = 0; j < calibration::m_calibration.cols(); ++j)
        {
            calibration::m_calibration(i,j) = components.at(k++);
        }
    }

    ROS_INFO_STREAM("loaded calibration matrix from " << param_name);
}
void calibration::update(const Eigen::Matrix4d& new_transform)
{
    calibration::m_calibration = new_transform;
}

// CALIBRATION
void calibration::calibrate(double& x, double& y, double& z)
{
    // Store point into vector.
    calibration::m_u(0) = x;
    calibration::m_u(1) = y;
    calibration::m_u(2) = z;

    // Calibrate point.
    calibration::m_c.noalias() = calibration::m_calibration * calibration::m_u;

    // Output calibrated point.
    x = calibration::m_c(0);
    y = calibration::m_c(1);
    z = calibration::m_c(2);
}
