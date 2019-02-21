/**
 * @file
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Implementation of the UAV dynamics and imu simulation
 */

#include "flightgoggles_uav_dynamics_node.hpp"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "flightgoggles_uav_dynamics_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Init class
  Uav_Dynamics uav_dynamics_node(n);

  // Spin
  ros::spin();


  return 0;
}


/**
 * @brief Constuctor
 * @param nh ROS node handle
 */
Uav_Dynamics::Uav_Dynamics(ros::NodeHandle nh):
// Node handle
node_(nh),
// TF listener
tfListener_(tfBuffer_),
// Initial pose vector
initPose_(7,0)
{
  // boost::shared_ptr<geometry_msgs::Pose const> initialPose = ros::topic::waitForMessage<geometry_msgs::Pose>("challenge/randStartPos"); 	
  //  Populate params
  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/ignore_collisions", ignoreCollisions_)){
      std::cout << "could not get param" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/vehicle_mass", vehicleMass_)) { 
      std::cout << "Did not get the vehicle mass from the params, defaulting to 1kg" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/motor_time_constant", motorTimeconstant_)) { 
      std::cout << "Did not get the motor time constant from the params, defaulting to 0.02" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/moment_arm", momentArm_)) { 
      std::cout << "Did not get the moment arm from the params, defaulting to 0.08" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/thrust_coefficient", thrustCoeff_)) { 
      std::cout << "Did not get the thrust coefficient from the params, defaulting to 1.91e-6" << std::endl;
  }
  
  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/torque_coefficient", torqueCoeff_)) { 
      std::cout << "Did not get the torque coefficient from the params, defaulting to 2.6e-7" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/drag_coefficient", dragCoeff_)) { 
      std::cout << "Did not get the drag coefficient from the params, defaulting to 0.1" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/vehicle_inertia_xx", vehicleInertia_[0])) { 
      std::cout << "Did not get the inertia (x) from the params, defaulting to 0.0049 kg m^2" << std::endl;
  }
  
  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/vehicle_inertia_yy", vehicleInertia_[1])) { 
      std::cout << "Did not get the inertia (y) from the params, defaulting to 0.0049 kg m^2" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/vehicle_inertia_zz", vehicleInertia_[2])) { 
      std::cout << "Did not get the inertia (z) from the params, defaulting to 0.0069 kg m^2" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/max_prop_speed", maxPropSpeed_)) { 
      std::cout << "Did not get the max prop speed from the params, defaulting to 2200 rad/s" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/angular_process_noise", angAccelProcessNoiseAutoCorrelation_)) { 
      std::cout << "Did not get the angular process noise from the params, defaulting to 0.00025 rad^2/s^2" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/linear_process_noise", linAccelProcessNoiseAutoCorrelation_)) { 
      std::cout << "Did not get the linear process noise from the params, defaulting to 0.0005 m^2/s^3" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/reset_timeout", resetTimeout_)) {
      std::cout << "Did not value for reset timeout from the params, defaulting to 0.1 sec" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/min_arming_thrust", minArmingThrust_)) {
    std::cout << "Did not value for minimum arming thrust from the params, defaulting to 9.9 N" << std::endl;
  }

  if (!ros::param::get("/use_sim_time", useSimTime_)) {
      std::cout << "Did not get bool useSimTime_ from the params, defaulting to false" << std::endl;
  }

  // Only enable clock scaling when simtime is enabled.
  if (useSimTime_) {
    if (!ros::param::get("/uav/flightgoggles_uav_dynamics/clockscale", clockScale)) {
      std::cout << "Using sim_time and did not get a clock scaling value. Defaulting to automatic clock scaling." << std::endl;
      useAutomaticClockscale_ = true;
    }
  }

  if (!ros::param::get("/uav/flightgoggles_uav_dynamics/init_pose", initPose_)) {
    // Start a few meters above the ground.
    std::cout << "Did NOT find initial pose from param file" << std::endl;

     initPose_.at(2) = 1.5;
     initPose_.at(6) = 1.0;
  }

  std::cout << "Ignore collisions: " << ignoreCollisions_ << std::endl;

  std::cout << "Initial pose: ";
  for (double i : initPose_){
    std::cout << i << " ";
  }
  std::cout << std::endl;

//  geometry_msgs::Pose initialPose;

  // Set z height to non-zero
//  initialPose.position.z = 1.0f;
//  initialPose.position.y = 2.0f;

  position_[0] = initPose_.at(0);
  position_[1] = initPose_.at(1);
  position_[2] = initPose_.at(2);

  attitude_[0] = initPose_.at(3);
  attitude_[1] = initPose_.at(4);
  attitude_[2] = initPose_.at(5);
  attitude_[3] = initPose_.at(6);

  for (size_t i = 0; i<4; i++){
    propSpeed_[i] = sqrt(vehicleMass_/4.*grav_/thrustCoeff_);
  }

  ////////////////// Init subscribers and publishers
  // Allow for up to 100ms sim time buffer of outgoing IMU messages. 
  // This should improve IMU integration methods on slow client nodes (see issue #63).
  imuPub_ = node_.advertise<sensor_msgs::Imu>("/uav/sensors/imu", 96);  
  inputCommandSub_ = node_.subscribe("/uav/input/rateThrust", 1, &Uav_Dynamics::inputCallback, this);
  collisionSub_ = node_.subscribe("/uav/collision", 1, &Uav_Dynamics::collisionCallback, this);
  frameRateSub_ = node_.subscribe("/uav/camera/debug/fps", 1, &Uav_Dynamics::fpsCallback, this);

  if (useSimTime_) {
    clockPub_ = node_.advertise<rosgraph_msgs::Clock>("/clock",1);
    clockPub_.publish(currentTime_);
  } else {
    // Get the current time if we are using wall time. Otherwise, use 0 as initial clock.
    currentTime_ = ros::Time::now();
  }

  timeLastReset_ = currentTime_;

  // Init main simulation loop at 2x framerate.
  simulationLoopTimer_ = node_.createWallTimer(ros::WallDuration(dt_secs/clockScale), &Uav_Dynamics::simulationLoopTimerCallback, this);
  simulationLoopTimer_.start();
}

/**
 * Callback to handle the frame rate from unity
 * @param msg Float msg of frame rate in sim time from unity
 */
void Uav_Dynamics::fpsCallback(std_msgs::Float32::Ptr msg) { 
  actualFps = msg->data; 
}



/**
 * Main Simulator loop
 * @param event Wall clock timer event
 */
void Uav_Dynamics::simulationLoopTimerCallback(const ros::WallTimerEvent& event){
  // Step the time forward
  if (useSimTime_){
    currentTime_ += ros::Duration(dt_secs);
    clockPub_.publish(currentTime_);
  } else {
      ros::Time loopStartTime = ros::Time::now();
      dt_secs = (loopStartTime - currentTime_).toSec();
      currentTime_ = loopStartTime;
  }

  if(hasCollided_ && !ignoreCollisions_){
    lpf_.resetState();
    pid_.resetState();
    resetState();
    hasCollided_ = false;
    armed_= false;
    timeLastReset_ = currentTime_;
    return;
  }

  // Only propagate simulation after have received input message
  if (armed_) {

    lpf_.proceedState(imuMeasurement_.angular_velocity, dt_secs);

    pid_.controlUpdate(lastCommandMsg_->angular_rates, lpf_.filterState_,
                       lpf_.filterStateDer_, angAccCommand_, dt_secs);
    computeMotorSpeedCommand();
    proceedState();
    imu_.getMeasurement(imuMeasurement_, angVelocity_, specificForceBodyFrame_, currentTime_);
    imuPub_.publish(imuMeasurement_);
  }


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
 * inputCallback to handle incoming rate thrust message
 * @param msg rateThrust message from keyboard/ joystick controller
 */
void Uav_Dynamics::inputCallback(mav_msgs::RateThrust::Ptr msg){
	lastCommandMsg_ = msg;
	if (!armed_ && ((currentTime_.toSec() - timeLastReset_.toSec()) > resetTimeout_)) { 
		if (msg->thrust.z >= (minArmingThrust_))
			armed_ = true;
	}
}

/**
 * collisionCallback to handle the checking of collisions
 * @param msg Empty message, this will be recieved when a collision is detected
 */
void Uav_Dynamics::collisionCallback(std_msgs::Empty::Ptr msg){
  hasCollided_ = true;
}

/**
 * computeMotorSpeedCommand computes the current motor speed
 */
void Uav_Dynamics::computeMotorSpeedCommand(void){
  double momentThrust[4] = {
    vehicleInertia_[0]*angAccCommand_[0],
    vehicleInertia_[1]*angAccCommand_[1],
    vehicleInertia_[2]*angAccCommand_[2],
    lastCommandMsg_->thrust.z
  };

  double motorSpeedsSquared[4] = {
    momentThrust[0]/(4*momentArm_*thrustCoeff_)+ -momentThrust[1]/(4*momentArm_*thrustCoeff_)+ -momentThrust[2]/(4*torqueCoeff_)+ momentThrust[3]/(4*thrustCoeff_),
    momentThrust[0]/(4*momentArm_*thrustCoeff_)+  momentThrust[1]/(4*momentArm_*thrustCoeff_)+  momentThrust[2]/(4*torqueCoeff_)+ momentThrust[3]/(4*thrustCoeff_),
   -momentThrust[0]/(4*momentArm_*thrustCoeff_)+  momentThrust[1]/(4*momentArm_*thrustCoeff_)+ -momentThrust[2]/(4*torqueCoeff_)+ momentThrust[3]/(4*thrustCoeff_),
   -momentThrust[0]/(4*momentArm_*thrustCoeff_)+ -momentThrust[1]/(4*momentArm_*thrustCoeff_)+  momentThrust[2]/(4*torqueCoeff_)+ momentThrust[3]/(4*thrustCoeff_)
  };

  //std::cout << motorSpeedsSquared[0] << "," << motorSpeedsSquared[1] << "," << motorSpeedsSquared[2] << "," << motorSpeedsSquared[3] << std::endl;

  // Assuming rotor speeds >= 0
  for(size_t i = 0; i < 4; i++)
    propSpeedCommand_[i] = fmin(sqrt(fmax(0.,motorSpeedsSquared[i])),maxPropSpeed_);
}


/**
 * proceedState compute first order euler integration of the state given inputs
 */
void Uav_Dynamics::proceedState(void){

  // Explicit Euler integration

  for(size_t i = 0; i < 3; i++){
    position_[i] += dt_secs*velocity_[i];
    velocity_[i] += dt_secs*specificForce_[i];
  }
  velocity_[2] -= dt_secs*grav_;

  double attitudeDer[4];
  attitudeDer[0] = 0.5*(angVelocity_[0]*attitude_[3] + angVelocity_[2]*attitude_[1] - angVelocity_[1]*attitude_[2]);
  attitudeDer[1] = 0.5*(angVelocity_[1]*attitude_[3] - angVelocity_[2]*attitude_[0] + angVelocity_[0]*attitude_[2]);
  attitudeDer[2] = 0.5*(angVelocity_[2]*attitude_[3] + angVelocity_[1]*attitude_[0] - angVelocity_[0]*attitude_[1]);
  attitudeDer[3] = 0.5*(-angVelocity_[0]*attitude_[0] - angVelocity_[1]*attitude_[1] - angVelocity_[2]*attitude_[2]);

  for(size_t i = 0; i < 4; i++)
    attitude_[i] += dt_secs*attitudeDer[i];

  double attNorm = sqrt(pow(attitude_[0],2.) + pow(attitude_[1],2.) + pow(attitude_[2],2.) + pow(attitude_[3],2.));

  for(size_t i = 0; i < 4; i++)
    attitude_[i] /= attNorm;

  double angAccelProcessNoise[3] = {
    sqrt(angAccelProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
    sqrt(angAccelProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
    sqrt(angAccelProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_)
  };

  double linAccelProcessNoise[3] = {
    sqrt(linAccelProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
    sqrt(linAccelProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
    sqrt(linAccelProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_)
  };

  double angAccel[3] = {
    momentArm_*thrustCoeff_/vehicleInertia_[0]*(pow(propSpeed_[0],2.) +pow(propSpeed_[1],2.) -pow(propSpeed_[2],2.) -pow(propSpeed_[3],2.)) + angAccelProcessNoise[0],
    momentArm_*thrustCoeff_/vehicleInertia_[1]*(-pow(propSpeed_[0],2.) +pow(propSpeed_[1],2.) +pow(propSpeed_[2],2.) -pow(propSpeed_[3],2.)) + angAccelProcessNoise[1],
    torqueCoeff_/vehicleInertia_[2]*(-pow(propSpeed_[0],2.) +pow(propSpeed_[1],2.) -pow(propSpeed_[2],2.) +pow(propSpeed_[3],2.)) + angAccelProcessNoise[2]
  };

  for(size_t i = 0; i < 3; i++)
    angVelocity_[i] += dt_secs*angAccel[i];

  for(size_t i = 0; i < 4; i++)
    propSpeed_[i] += fmin(dt_secs,motorTimeconstant_)*((propSpeedCommand_[i] - propSpeed_[i])/motorTimeconstant_);

  double specificThrust = thrustCoeff_*(pow(propSpeed_[0],2.) + pow(propSpeed_[1],2.) + pow(propSpeed_[2],2.) + pow(propSpeed_[3],2.))/vehicleMass_;
  
  double speed = sqrt(pow(velocity_[0],2.) + pow(velocity_[1],2.) + pow(velocity_[2],2.));
  if (includeDrag_ && (speed > 0.)){
    double drag = dragCoeff_*pow(speed,2.);

    double dragForce[3];
    dragForce[0] = drag*-velocity_[0]/speed + linAccelProcessNoise[0];
    dragForce[1] = drag*-velocity_[1]/speed + linAccelProcessNoise[1];
    dragForce[2] = drag*-velocity_[2]/speed + linAccelProcessNoise[2];

    double a = attitude_[3];
    double b = attitude_[0];
    double c = attitude_[1];
    double d = attitude_[2];

    specificForceBodyFrame_[0] = (pow(a,2.) + pow(b,2.) - pow(c,2.) - pow(d,2.))*dragForce[0] +
                                 (2*b*c +2*a*d)*dragForce[1] +
                                 (2*b*d - 2*a*c)*dragForce[2];
    specificForceBodyFrame_[1] = (2*b*c - 2*a*d)*dragForce[0] +
                                 (pow(a,2.) - pow(b,2.) + pow(c,2.) - pow(d,2.))*dragForce[1] +
                                 (2*c*d + 2*a*b)*dragForce[2];
    specificForceBodyFrame_[2] = (2*b*d + 2*a*c)*dragForce[0] +
                                 (2*c*d - 2*a*b)*dragForce[1] +
                                 (pow(a,2.) - pow(b,2.) - pow(c,2.) + pow(d,2.))*dragForce[2];

    specificForce_[0] = dragForce[0];
    specificForce_[1] = dragForce[1];
    specificForce_[2] = dragForce[2];
  }
  else{
    double a = attitude_[3];
    double b = attitude_[0];
    double c = attitude_[1];
    double d = attitude_[2];

    specificForceBodyFrame_[0] = (pow(a,2.) + pow(b,2.) - pow(c,2.) - pow(d,2.))*linAccelProcessNoise[0] +
                                 (2*b*c +2*a*d)*linAccelProcessNoise[1] +
                                 (2*b*d - 2*a*c)*linAccelProcessNoise[2];
    specificForceBodyFrame_[1] = (2*b*c - 2*a*d)*linAccelProcessNoise[0] +
                                 (pow(a,2.) - pow(b,2.) + pow(c,2.) - pow(d,2.))*linAccelProcessNoise[1] +
                                 (2*c*d + 2*a*b)*linAccelProcessNoise[2];
    specificForceBodyFrame_[2] = (2*b*d + 2*a*c)*linAccelProcessNoise[0] +
                                 (2*c*d - 2*a*b)*linAccelProcessNoise[1] +
                                 (pow(a,2.) - pow(b,2.) - pow(c,2.) + pow(d,2.))*linAccelProcessNoise[2];
    specificForce_[0] = linAccelProcessNoise[0];
    specificForce_[1] = linAccelProcessNoise[1];
    specificForce_[2] = linAccelProcessNoise[2];
  }

  specificForceBodyFrame_[2] += specificThrust;

  specificForce_[0] += (2.*attitude_[0]*attitude_[2] + 2.*attitude_[1]*attitude_[3])*specificThrust;
  specificForce_[1] += (2.*attitude_[1]*attitude_[2] - 2.*attitude_[0]*attitude_[3])*specificThrust;
  specificForce_[2] += (-pow(attitude_[0],2.) - pow(attitude_[1],2.) + pow(attitude_[2],2.) + pow(attitude_[3],2.))*specificThrust;
}

/**
 * resetState reset state to initial
 */
void Uav_Dynamics::resetState(void){
  position_[0] = initPose_.at(0);
  position_[1] = initPose_.at(1);
  position_[2] = initPose_.at(2);

  attitude_[0] = initPose_.at(3);
  attitude_[1] = initPose_.at(4);
  attitude_[2] = initPose_.at(5);
  attitude_[3] = initPose_.at(6);

  for (size_t i = 0; i<3; i++){
    angVelocity_[i] = 0.;
    velocity_[i] = 0.;
    specificForce_[i] = 0.;
    propSpeed_[i] = sqrt(vehicleMass_/4.*grav_/thrustCoeff_);
  }
  propSpeed_[3] = sqrt(vehicleMass_/4.*grav_/thrustCoeff_);

  specificForce_[2] = 9.81;
}

/**
 * publishState publishes state transform message
 */
void Uav_Dynamics::publishState(void){
  geometry_msgs::TransformStamped transform;

  transform.header.stamp = currentTime_;
  transform.header.frame_id = "world";

  transform.transform.translation.x = position_[0];
  transform.transform.translation.y = position_[1];
  transform.transform.translation.z = position_[2];

  transform.transform.rotation.x = attitude_[0];
  transform.transform.rotation.y = attitude_[1];
  transform.transform.rotation.z = attitude_[2];
  transform.transform.rotation.w = attitude_[3];

  transform.child_frame_id = "uav/imu";

  tfPub_.sendTransform(transform);
}

/**
 * Constructor for Uav_Imu
 */
Uav_Imu::Uav_Imu(){
  if (!ros::param::get("/uav/flightgoggles_imu/accelerometer_variance", accMeasNoiseVariance_)) { 
      std::cout << "Did not get the accelerometer variance from the params, defaulting to 0.005 m^2/s^4" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_imu/gyroscope_variance", gyroMeasNoiseVariance_)) { 
      std::cout << "Did not get the gyroscope variance from the params, defaulting to 0.003 rad^2/s^2" << std::endl;
  }
}

/**
 * getMeasurement helper function to get the simulated imu measurement
 * @param meas message to be filled up
 * @param angVel angular velocity buffer
 * @param accel acceleration buffer
 * @param currTime current simulation time
 */
void Uav_Imu::getMeasurement(sensor_msgs::Imu & meas, double * angVel,
                        double * accel, ros::Time currTime){
  meas.header.stamp = currTime;
  meas.orientation_covariance[0] = -1;

  meas.angular_velocity.x = angVel[0] + sqrt(gyroMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);
  meas.linear_acceleration.x = accel[0] + sqrt(accMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);

  meas.angular_velocity.y = angVel[1] + sqrt(gyroMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);
  meas.linear_acceleration.y = accel[1] + sqrt(accMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);

  meas.angular_velocity.z = angVel[2] + sqrt(gyroMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);
  meas.linear_acceleration.z = accel[2] + sqrt(accMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);

  meas.angular_velocity_covariance[0] = gyroMeasNoiseVariance_;
  meas.linear_acceleration_covariance[0] = accMeasNoiseVariance_;
  for (size_t i = 1; i < 8; i++){
    if (i == 4){
      meas.angular_velocity_covariance[i] = gyroMeasNoiseVariance_;
      meas.linear_acceleration_covariance[i] = accMeasNoiseVariance_;
    }
    else{
      meas.angular_velocity_covariance[i] = 0.;
      meas.linear_acceleration_covariance[i] = 0.;
    }
    meas.angular_velocity_covariance[8] = gyroMeasNoiseVariance_;
    meas.linear_acceleration_covariance[8] = accMeasNoiseVariance_;
  }
}

/**
 * Constructor for Uav_LowPassFilter
 */
Uav_LowPassFilter::Uav_LowPassFilter(){

  double temp1;
  double temp2;

  if ((!ros::param::get("/uav/flightgoggles_lpf/gain_p", temp1)) ||
      (!ros::param::get("/uav/flightgoggles_lpf/gain_q", temp2)))
  { 
      std::cout << "Did not get the LPF gain_p and/or gain_q from the params, defaulting to 30 Hz cutoff freq." << std::endl;
  }
  else
  {
    gainP_ = temp1;
    gainQ_ = temp2;
  }
}

/**
 * proceedState propagates the state
 * @param value
 * @param dt
 */
void Uav_LowPassFilter::proceedState(geometry_msgs::Vector3 & value, double dt){
  double input[] = {value.x, value.y, value.z};

  double det = gainP_ * dt * dt + gainQ_ * dt + 1.;
  double stateDer;
  for (size_t ind = 0; ind < 3; ind++) {
    stateDer = (filterStateDer_[ind] + gainP_ * dt * input[ind]) / det -
               (dt * gainP_ * filterState_[ind]) / det;
    filterState_[ind] =
        (dt * (filterStateDer_[ind] + gainP_ * dt * input[ind])) / det +
        ((dt * gainQ_ + 1.) * filterState_[ind]) / det;
    filterStateDer_[ind] = stateDer;
  }
}

/**
 * resetState resets the state
 */
void Uav_LowPassFilter::resetState(void){
  for (size_t ind = 0; ind < 3; ind++) {
    filterState_[ind] = 0.;
    filterStateDer_[ind] = 0.;
  }
}

/**
 * Constructor for the Uav Pid
 */
Uav_Pid::Uav_Pid(){
  if (!ros::param::get("/uav/flightgoggles_pid/gain_p_roll", propGain_[0])) { 
      std::cout << "Did not get the gain p roll from the params, defaulting to 9.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_i_roll", intGain_[0])) { 
      std::cout << "Did not get the gain i roll from the params, defaulting to 3.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_d_roll", derGain_[0])) { 
      std::cout << "Did not get the gain d roll from the params, defaulting to 0.3" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_p_pitch", propGain_[1])) { 
      std::cout << "Did not get the gain p pitch from the params, defaulting to 9.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_i_pitch", intGain_[1])) { 
      std::cout << "Did not get the gain i pitch from the params, defaulting to 3.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_d_pitch", derGain_[1])) { 
      std::cout << "Did not get the gain d pitch from the params, defaulting to 0.3" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_p_yaw", propGain_[2])) { 
      std::cout << "Did not get the gain p yaw from the params, defaulting to 9.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_i_yaw", intGain_[2])) { 
      std::cout << "Did not get the gain i yaw from the params, defaulting to 3.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_d_yaw", derGain_[2])) { 
      std::cout << "Did not get the gain d yaw from the params, defaulting to 0.3" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/int_bound_roll", intBound_[0])) { 
      std::cout << "Did not get the roll integrator bound from the params, defaulting to 1000.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/int_bound_pitch", intBound_[1])) { 
      std::cout << "Did not get the pitch integrator bound from the params, defaulting to 1000.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/int_bound_yaw", intBound_[2])) { 
      std::cout << "Did not get the yaw integrator bound from the params, defaulting to 1000.0" << std::endl;
  }
}

/**
 * controlUpdate
 * @param command
 * @param curval
 * @param curder
 * @param out
 * @param dt
 */
void Uav_Pid::controlUpdate(geometry_msgs::Vector3 & command, double * curval,
                      double * curder, double * out, double dt){

  double stateDev[] = {command.x-curval[0], command.y-curval[1], command.z-curval[2]};

  for (size_t ind = 0; ind < 3; ind++) {
    intState_[ind] += dt * stateDev[ind];
    intState_[ind] = fmin(fmax(-intBound_[ind],intState_[ind]),intBound_[ind]);
    out[ind] = propGain_[ind] * stateDev[ind] +
               intGain_[ind] * intState_[ind] + derGain_[ind] * -curder[ind];
  }
}

/**
 * resetState
 */
void Uav_Pid::resetState(void){
  for (size_t ind = 0; ind < 3; ind++) {
    intState_[ind] = 0.;
  }
}

