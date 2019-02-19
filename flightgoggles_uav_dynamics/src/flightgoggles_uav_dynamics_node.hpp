/**
 * @file flightgoggles_uav_dynamics.hpp
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Header file for UAV dynamics and imu simulation
 */

#ifndef UAV_DYNAMICS_HPP
#define UAV_DYNAMICS_HPP

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// Messages
#include <mav_msgs/RateThrust.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <rosgraph_msgs/Clock.h>

#include <sstream>
#include <random>

class Uav_Imu {
    // This class can be extended if IMU noise is needed etc.
    public:
        /// @name Constructor
        Uav_Imu();
        /// @name Accessors
        void getMeasurement(sensor_msgs::Imu & meas,
            double * angVel, double * accel, ros::Time currTime);
        
    private:
        /// @name Noise Generation Helpers
        //@{
        std::default_random_engine randomNumberGenerator_;
        std::normal_distribution<double> standardNormalDistribution_ = std::normal_distribution<double>(0.0,1.0);
        double accMeasNoiseVariance_ = 0.005; // m^2/s^4
        double gyroMeasNoiseVariance_ = 0.003; // rad^2/s^2
        //@}
};

class Uav_LowPassFilter{
    public:
        /// @name Constructor
        Uav_LowPassFilter();
        void proceedState(geometry_msgs::Vector3 & value, double dt);
        void resetState(void);

        /// @name Low-Pass Filter State Variables
        //@{
        double filterState_[3] = {0.,0.,0.};
        double filterStateDer_[3] = {0.,0.,0.};
        //@}

    private:
        /// @name Low-Pass Filter Gains
        //@{
        double gainP_ = 35530.5758439217;
        double gainQ_ = 266.572976289502;
        //@}
};

class Uav_Pid {
    public:
        /// @name Constructor
        Uav_Pid();
        void controlUpdate(geometry_msgs::Vector3 & command, double * curval,
                      double * curder, double * out, double dt);
        void resetState(void);

    private:
        /// @name PID Controller Parameters
        //@{
        double propGain_[3] = {9.0, 9.0, 9.0};
        double intGain_[3] = {3.0, 3.0, 3.0};
        double derGain_[3] = {0.3, 0.3, 0.3};
        double intState_[3] = {0.,0.,0.};
        double intBound_[3] = {1000.,1000.,1000.};
        //@}
};

class Uav_Dynamics {
    public:
        /// @name Constructor
        // Requires nodehandle
        Uav_Dynamics(ros::NodeHandle nh);
        
        /// @name Node handle
        ros::NodeHandle node_;

        /// @name Transform Publishers and Subscribers
        //@{
        tf2_ros::TransformBroadcaster tfPub_;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;
        //@}

        /// @name Publishers
        //@{
        ros::Publisher imuPub_;
        ros::Publisher clockPub_;
        //@}

        /// @name Subscribers
        //@{
        ros::Subscriber inputCommandSub_;
        ros::Subscriber collisionSub_;
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
        void collisionCallback(std_msgs::Empty::Ptr msg);
	    void fpsCallback(std_msgs::Float32::Ptr msg);
        //@}

        /// @name Storage Variables
        //@{
        mav_msgs::RateThrust::Ptr lastCommandMsg_;
        sensor_msgs::Imu imuMeasurement_;
        ros::Time currentTime_;
        ros::Time timeLastReset_;
        //@}

        double dt_secs = 1.0f/960.;
        bool useAutomaticClockscale_ = false;
        double clockScale = 1.0;
        double actualFps  = -1;

        double resetTimeout_ = 0.1;
        // Min input thrust required before drone is allowed to take off.
        double minArmingThrust_ = 9.9; // Newtons


    private:

        void computeMotorSpeedCommand(void);
        void proceedState(void);
        void resetState(void);
        void publishState(void);

        Uav_Imu imu_;
        Uav_Pid pid_;
        Uav_LowPassFilter lpf_;

        /// @name UAV Dynamics Flags
        //@{
        bool includeDrag_ = true;
        bool hasCollided_ = false;
        bool ignoreCollisions_ = false;
	    bool useSimTime_ = false;
	    bool armed_ = false;
        //@}

        /* Everything defined as standard in ROS:
        x : forward
        y : leftward
        z : upward */

        /* Motors are numbered counterclockwise (look at quadcopter from above) with
           motor 1 in the positive quadrant of the X-Y plane (i.e. front left). */
       
        /// @name Vehicle properties
        //@{
        double vehicleMass_ = 1.0; // kg
        double vehicleInertia_[3] = {0.0049,0.0049,0.0069}; // kg m^2
        double motorTimeconstant_ = 0.02; // s
        double maxPropSpeed_ = 2200.; // rad/s per motor
        double momentArm_ = 0.08; // m moment arm from motor position to cog
        const double grav_ = 9.81; // m/s^2
        double thrustCoeff_ = 1.91e-6; // N/(rad/s)^2
        double torqueCoeff_ = 2.6e-7; // Nm/(rad/s)^2
        double dragCoeff_ = 0.1; // N/(m/s)^2
        //@}

        /// @name State variables
        //@{
        std::vector<double> initPose_;

        double position_[3]; // m
        double attitude_[4]; // quaternion [x,y,z,w]
        double angVelocity_[3] = {0.,0.,0.}; // rad/s
        double velocity_[3] = {0.,0.,0.}; // m/s
        double specificForce_[3] = {0.,0.,9.81}; // N/kg or m/s^2
        double specificForceBodyFrame_[3]; // N/kg or m/s^2
        double propSpeed_[4]; // rad/s
        //@}

        /// @name Standard normal distrubution RNG
        //@{
        std::default_random_engine randomNumberGenerator_;
        std::normal_distribution<double> standardNormalDistribution_ = std::normal_distribution<double>(0.0,1.0);
        double angAccelProcessNoiseAutoCorrelation_ = 0.00025; // rad^2/s^2
        double linAccelProcessNoiseAutoCorrelation_ = 0.0005; // m^2/s^3
        //@}

        /// @name Control
        //@{
        double angAccCommand_[3];
        double propSpeedCommand_[4];
        //@}
};

#endif
