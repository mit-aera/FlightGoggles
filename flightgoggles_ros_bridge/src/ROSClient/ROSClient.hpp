#ifndef ROSCLIENT_HPP
#define ROSCLIENT_HPP
/**
 * @file   ROSClient.hpp
 * @author Winter Guerra
 * @brief  Basic ros client interface for FlightGoggles.
 */

#include <FlightGogglesClient.hpp>

#include <iostream>
#include <cmath>
#include <random>
#include <algorithm>
#include <string>
#include <vector>
#include <thread>
#include <boost/algorithm/string.hpp>

#include "ros/ros.h"
#include <ros/time.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float32.h"
#include "flightgoggles/IRMarkerArray.h"
#include "geometry_msgs/Pose.h"
#include "tf2_msgs/TFMessage.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

class ROSClient {
public:
    /// @name FlightGoggles interface object
	FlightGogglesClient flightGoggles;

	/// @name Node handles for ROS
	//@{
	ros::NodeHandle ns_;
    ros::NodeHandle nsPrivate_;
    //@}

	/// @name Transform listeners for listening to ROS TF
	//@{
	tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener tfListener_;
	//@}

	/// @name Subscribers
	//@{
    ros::Subscriber tfSubscriber_;
    ros::Subscriber irSubscriber_;
    //@}

    /// @name Image publishers
    //@{
	image_transport::ImageTransport it_;
	image_transport::CameraPublisher imagePubLeft_;
	image_transport::CameraPublisher imagePubRight_;
	//@}

	/// @name Topic publishers
	//@{
	ros::Publisher collisionPub_;
    ros::Publisher lidarPub_;
    ros::Publisher irMarkerPub_;
    ros::Publisher fpsPublisher_;
    //@}

    //// @name State variables
    //@{
    bool render_stereo = false;
    int numSimulationStepsSinceLastRender_ = 0;
    const int numSimulationStepsBeforeRenderRequest_ = 15;
//    sensor_msgs::PointCloud2::Ptr irBeaconGroundTruth_;
    sensor_msgs::CameraInfo cameraInfoLeft;
	sensor_msgs::CameraInfo cameraInfoRight;
	float baseline_ = 0.32;
	int imageWidth_ = 1024;
	int imageHeight_ = 768;

	// Lidar params
	float lidarMaxRange_ = 20; // Meters
    float lidarVariance_ = 0.0009; // Meters^2

    // Noise generators for lidar
    std::default_random_engine randomNumberGenerator_;
    std::normal_distribution<float> standardNormalDistribution_ = std::normal_distribution<float>(0.0,1.0);

    // Keep track of the frame rate
    ros::WallTime timeSinceLastMeasure_;
    int64_t frameCount_;
    //@}

    /// @name Transforms
    //@{
    geometry_msgs::TransformStamped imu_T_Camera_;
    //@}

    /// @name Constructor
	ROSClient(ros::NodeHandle nh, ros::NodeHandle nhPrivate);

	// Populate starting settings into state
	void populateRenderSettings();


	/// @name Callbacks
	void tfCallback(tf2_msgs::TFMessage::Ptr msg);
//    void irBeaconPointcloudCallback(sensor_msgs::PointCloud2::Ptr msg);


private:
};

#endif
