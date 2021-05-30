/**
 * @file flightgoggles_car_dynamics_node.cpp
 * @author Ezra Tal
 * @brief Implementation of car dynamics simulation node
 * 
 */

#include "flightgoggles_car_dynamics_node.hpp"

/**
 * @brief Global function designated start of car dynamics node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "flightgoggles_car_dynamics_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Init class
  Car_Dynamics car_dynamics_node(n);

  // Spin
  ros::spin();

  return 0;
}

/**
 * @brief Construct a new Car_Dynamics::Car_Dynamics object
 * 
 * @param nh ROS Nodehandle
 */
Car_Dynamics::Car_Dynamics(ros::NodeHandle nh):
// Node handle
node_(nh)
{
  //  Populate params
  if (!ros::param::get("/uav/flightgoggles_car_dynamics/ignore_collisions", ignoreCollisions_)){
      std::cout << "Did not get bool ignoreCollisions_ from the params, defaulting to false" << std::endl;
  }

  if (!ros::param::get("/use_sim_time", useSimTime_)) {
      std::cout << "Did not get bool useSimTime_ from the params, defaulting to false" << std::endl;
  }

  double maxSpeed;
  if (!ros::param::get("/uav/flightgoggles_car_dynamics/max_speed", maxSpeed)) { 
      std::cout << "Did not get the max_speed from the params, defaulting to 5.0" << std::endl;
      maxSpeed = 5.;
  }

  double minSpeed;
  if (!ros::param::get("/uav/flightgoggles_car_dynamics/min_speed", minSpeed)) { 
      std::cout << "Did not get the min_speed from the params, defaulting to 0.0" << std::endl;
      minSpeed = 0.;
  }

  double maxSteeringAngle;
  if (!ros::param::get("/uav/flightgoggles_car_dynamics/max_steering_angle", maxSteeringAngle)) { 
      std::cout << "Did not get the max_steering_angle from the params, defaulting to 1.0" << std::endl;
      maxSteeringAngle = 1.;
  }

  std::vector<double> initPosition(2);
  if (!ros::param::get("/uav/flightgoggles_car_dynamics/init_position", initPosition)) {
    std::cout << "Did NOT find initial position from param file" << std::endl;
    initPosition.at(0) = 0.;
    initPosition.at(1) = 0.;
  }

  double initHeading;
  if (!ros::param::get("/uav/flightgoggles_car_dynamics/init_heading", initHeading)) { 
      std::cout << "Did not get the init_heading from the param file" << std::endl;
      initHeading = 0.;
  }

  std::vector<double> velProcNoiseAutoCorr(2);
  if (!ros::param::get("/uav/flightgoggles_car_dynamics/velocity_process_noise", velProcNoiseAutoCorr)) {
    std::cout << "Did NOT find velocity process noise auto correlation from param file" << std::endl;
    velProcNoiseAutoCorr.at(0) = 0.;
    velProcNoiseAutoCorr.at(1) = 0.;
  }

  Eigen::Vector2d velProcNoiseAutoCorrVector(velProcNoiseAutoCorr.at(0), velProcNoiseAutoCorr.at(1));

  // Create car simulator
  carSim_ = new CarDynamicsSim(maxSteeringAngle, minSpeed, maxSpeed, velProcNoiseAutoCorrVector);

  double noise_seed;
  if (ros::param::get("/uav/flightgoggles_car_dynamics/noise_seed", noise_seed)) {
      carSim_->setNoiseSeed(static_cast<unsigned>(noise_seed));
  }
  else{
      std::cout << "Did not get the RNG noise seed from the param file, initialized using current time." << std::endl; 
  }

  // Set initial conditions
  initPosition_ << initPosition.at(0), initPosition.at(1);
  initHeading_ = Eigen::Rotation2Dd(initHeading);

  carSim_->setVehiclePosition(initPosition_);
  carSim_->setVehicleHeading(initHeading_);

  // Only enable clock scaling when simtime is enabled.
  if (useSimTime_) {
    if (!ros::param::get("/uav/flightgoggles_car_dynamics/clockscale", clockScale)) {
      std::cout << "Using sim_time and did not get a clock scaling value. Defaulting to automatic clock scaling." << std::endl;
      useAutomaticClockscale_ = true;
    }
  }

  // Print several parameters to terminal
  std::cout << "Ignore collisions: " << ignoreCollisions_ << std::endl;
  std::cout << "Initial position: " << initPosition_(0) << ", "
                                    << initPosition_(1) << std::endl;
  std::cout << "Initial heading:  " << initHeading << std::endl;

  // Init subscribers and publishers
  inputCommandSub_ = node_.subscribe("/uav/input/rateThrust", 1, &Car_Dynamics::inputCallback, this);
  collisionSub_ = node_.subscribe("/uav/collision", 1, &Car_Dynamics::collisionCallback, this);
  armSub_ = node_.subscribe("/uav/input/arm", 1, &Car_Dynamics::armCallback, this);
  resetSub_ = node_.subscribe("/uav/input/reset", 1, &Car_Dynamics::resetCallback, this);
  frameRateSub_ = node_.subscribe("/uav/camera/debug/fps", 1, &Car_Dynamics::fpsCallback, this);

  if (useSimTime_) {
    clockPub_ = node_.advertise<rosgraph_msgs::Clock>("/clock",1);
    clockPub_.publish(currentTime_);
  } else {
    // Get the current time if we are using wall time. Otherwise, use 0 as initial clock.
    currentTime_ = ros::Time::now();
  }

  // Init main simulation loop
  simulationLoopTimer_ = node_.createWallTimer(ros::WallDuration(dt_secs/clockScale), &Car_Dynamics::simulationLoopTimerCallback, this);
  simulationLoopTimer_.start();
}

/**
 * @brief Callback to handle the frame rate from unity
 * @param msg Float msg of frame rate in sim time from unity
 */
void Car_Dynamics::fpsCallback(std_msgs::Float32::Ptr msg) { 
  actualFps = msg->data; 
}

/**
 * @brief Main Simulator loop
 * @param event Wall clock timer event
 */
void Car_Dynamics::simulationLoopTimerCallback(const ros::WallTimerEvent& event){
  // Step the time forward
  if (useSimTime_){
    currentTime_ += ros::Duration(dt_secs);
    clockPub_.publish(currentTime_);
  } else {
      ros::Time loopStartTime = ros::Time::now();
      dt_secs = (loopStartTime - currentTime_).toSec();
      currentTime_ = loopStartTime;
  }

  // In case of collision reset state and disarm
  if(resetRequested_ || (hasCollided_ && !ignoreCollisions_)){
    resetState();
    lastCommandMsg_.reset();
    hasCollided_ = false;
    armed_= false;
    resetRequested_ = false;
    return;
  }

  // Only propagate simulation if armed
  if (armed_) {  

    // Proceed car dynamics
    carSim_->proceedState_ExplicitEuler(dt_secs, lastCommandMsg_->angular_rates.y, -lastCommandMsg_->angular_rates.x);
  }

  // Publish car state
  publishState();

  // Update clockscale if necessary
  if (actualFps != -1 && actualFps < 1e3 && useSimTime_ && useAutomaticClockscale_) {
     clockScale =  (actualFps / 55.0);
     simulationLoopTimer_.stop();
     simulationLoopTimer_.setPeriod(ros::WallDuration(dt_secs / clockScale));
     simulationLoopTimer_.start();
  }
}

/**
 * @brief Handle incoming rate thrust message
 * @param msg rateThrust message from keyboard/ joystick controller
 */
void Car_Dynamics::inputCallback(mav_msgs::RateThrust::Ptr msg){
	lastCommandMsg_ = msg;
}

/**
 * @brief Handle arming message
 * @param msg Empty message, this will be recieved when vehicle is to be armed
 */
void Car_Dynamics::armCallback(std_msgs::Empty::Ptr msg){
	armed_ = true;
}

/**
 * @brief Handle reset message
 * @param msg Empty message, this will be recieved when vehicle is to be reset
 */
void Car_Dynamics::resetCallback(std_msgs::Empty::Ptr msg){
	resetRequested_ = true;
}

/**
 * @brief Handle the checking of collisions
 * @param msg Empty message, this will be recieved when a collision is detected
 */
void Car_Dynamics::collisionCallback(std_msgs::Empty::Ptr msg){
  hasCollided_ = true;
}

/**
 * @brief Reset state to initial
 */
void Car_Dynamics::resetState(void){
  carSim_->setVehiclePosition(initPosition_);
  carSim_->setVehicleHeading(initHeading_);
}

/**
 * @brief Publish state transform message
 */
void Car_Dynamics::publishState(void){
  geometry_msgs::TransformStamped transform;

  transform.header.stamp = currentTime_;
  transform.header.frame_id = "world";

  Eigen::Vector2d position = carSim_->getVehiclePosition();
  Eigen::Rotation2Dd heading = carSim_->getVehicleHeading();

  transform.transform.translation.x = position(0);
  transform.transform.translation.y = position(1);
  transform.transform.translation.z = 0.;

  transform.transform.rotation.x = 0.;
  transform.transform.rotation.y = 0.;
  transform.transform.rotation.z = sin(heading.smallestAngle()*0.5);
  transform.transform.rotation.w = cos(heading.smallestAngle()*0.5);

  transform.child_frame_id = "uav/imu";

  tfPub_.sendTransform(transform);
}