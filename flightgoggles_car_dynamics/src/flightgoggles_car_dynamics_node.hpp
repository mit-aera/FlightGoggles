/**
 * @file flightgoggles_car_dynamics_node.hpp
 * @author Ezra Tal
 * @brief Header file for car dynamics simulation node
 * 
 */

#ifndef CAR_DYNAMICS_HPP
#define CAR_DYNAMICS_HPP

// ROS includes
#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// Messages
#include <mav_msgs/RateThrust.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <rosgraph_msgs/Clock.h>

#include "../libs/carDynamicsSim/carDynamicsSim.hpp"

#include <sstream>
#include <random>

/**
 * @brief Car Dynamics class used for dynamics simulation node
 * 
 */
class Car_Dynamics {
    public:
        /// @name Constructor
        Car_Dynamics(ros::NodeHandle nh);
        
        /// @name Node handle
        ros::NodeHandle node_;

        /// @name Transform Publishers
        //@{
        tf2_ros::TransformBroadcaster tfPub_;
        //@}

        /// @name Publishers
        //@{
        ros::Publisher clockPub_;

        void publishState(void);
        //@}

        /// @name Subscribers
        //@{
        ros::Subscriber inputCommandSub_;
        ros::Subscriber collisionSub_;
        ros::Subscriber armSub_;
        ros::Subscriber resetSub_;
	    ros::Subscriber frameRateSub_;
        //@}

        /// @name Timers
        //@{
        ros::WallTimer simulationLoopTimer_;
        //@}

        /// @name Callbacks
        //@{
        void simulationLoopTimerCallback(const ros::WallTimerEvent& event);
        void inputCallback(mav_msgs::RateThrust::Ptr msg);
        void armCallback(std_msgs::Empty::Ptr msg);
        void resetCallback(std_msgs::Empty::Ptr msg);
        void collisionCallback(std_msgs::Empty::Ptr msg);
	    void fpsCallback(std_msgs::Float32::Ptr msg);
        //@}

        /// @name Control inputs
        //@{
        mav_msgs::RateThrust::Ptr lastCommandMsg_;
        //@}

        /// @name Time keeping variables
        //@{
        ros::Time currentTime_;
        double dt_secs = 1.0f/960.;
        bool useAutomaticClockscale_ = false;
        double clockScale = 1.0;
        double actualFps  = -1;
        bool useSimTime_ = false;
        //@}

        /// @name Simulation state control parameters and flags
        //@{
        bool hasCollided_ = false;
        bool ignoreCollisions_ = false;
	    bool armed_ = false;
        bool resetRequested_ = false;
        //@}

        void resetState(void);


        /// @name Vehicle dynamics simulator
        //@{
        CarDynamicsSim *carSim_;
        //@}

        /// @name Initial conditions
        //@{
        Eigen::Vector2d initPosition_;
        Eigen::Rotation2Dd initHeading_;
        //@}
};

#endif
