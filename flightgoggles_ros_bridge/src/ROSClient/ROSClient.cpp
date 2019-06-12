/**
 * @file   ROSClient.cpp
 * @author Winter Guerra
 * @brief  Pulls images from Unity and publish them as ROS images.
 *
 **/

#include "ROSClient.hpp"

#define SHOW_DEBUG_IMAGE_FEED false

/// Constructor
ROSClient::ROSClient(ros::NodeHandle ns, ros::NodeHandle nhPrivate):
    // Node handle
    ns_(ns),
    nsPrivate_(nhPrivate),
    // TF listener
    tfListener_(tfBuffer_),
    //Image transport
    it_(ns_)
{


    // Get Global render/scene configs
    if (!ros::param::get("/uav/flightgoggles_ros_bridge/scene_filename", sceneFilename_)) {
        ROS_INFO("Did not get argument for scene_filename. Defaulting to Abandoned_Factory_Morning");
    }

    if (!ros::param::get("/uav/flightgoggles_ros_bridge/body_frame", bodyFrame_)) {
        ROS_INFO( "Did not get argument for body_frame. Defaulting to uav/imu" );
    }
    
    if (!ros::param::get("/uav/flightgoggles_ros_bridge/world_frame", worldFrame_)) {
        ROS_INFO( "Did not get argument for world_frame. Defaulting to world/ned" );
    }
    
    if (!ros::param::get("/uav/flightgoggles_ros_bridge/image_width", imageWidth_)) {
        ROS_INFO( "Did not get argument for image width. Defaulting to 1024 px" );
    }

    
    if (!ros::param::get("/uav/flightgoggles_ros_bridge/image_height", imageHeight_)) {
        ROS_INFO( "Did not get argument for image height. Defaulting to 768 px" );
    }

    if (!ros::param::get("/uav/flightgoggles_ros_bridge/framerate", framerate_)) {
        ROS_INFO( "Did not get argument for framerate. Defaulting to 60Hz" );
    }
    
    if (!ros::param::get("/uav/flightgoggles_laser/rangefinder_max_range", lidarMaxRange_)) {
        ROS_INFO( "Did not get argument for rangefinder max range. Defaulting to 20 m" );
    }

    if (!ros::param::get("/uav/flightgoggles_laser/rangefinder_variance", lidarVariance_)) {
        ROS_INFO( "Did not get argument for rangefinder variance. Defaulting to 0.009 m^2" );
    }

    if (!ros::param::get("/uav/flightgoggles_ros_bridge/obstacle_perturbation_file", obstaclePerturbationFile_)) {
        ROS_INFO( "Did not get argument for obstacle_perturbation_file. Defaulting to no offsets." );
    }

    
    // Load list of obstacles to spawn.
    if (!ros::param::get("/uav/flightgoggles_ros_bridge/obstacle_frames", obstacleTFList_)) {
        ROS_INFO( "Did not get argument for obstacle_frames. Defaulting to using no dynamic obstacles." );
    }
    // Load list of obstacles to ignore
    if (!ros::param::get("/uav/flightgoggles_ros_bridge/ignored_obstacle_frames", obstacleIgnoreList_)) {
        ROS_INFO( "Did not get argument for ignored_obstacle_frames. Defaulting to ignoring body_frame object" );
        obstacleIgnoreList_.push_back(bodyFrame_);
    }

    // Remove ignored obstacles from list of obstacles to spawn
    for (auto ignoredObstacle : obstacleIgnoreList_){
      auto itr = std::find(obstacleTFList_.begin(), obstacleTFList_.end(), ignoredObstacle);
      if (itr != obstacleTFList_.end()) obstacleTFList_.erase(itr);
    }

    // Get list of cameras
    if (!ros::param::get("/sensors/camera/camera_list", cameraNameList_)) {
        ROS_ERROR( "Did not get list of cameras! This must be defined in /sensors/camera/camera_list." );
        
    }

    // Load list of timestamps to render (if running from rosbag)
    if (!ros::param::get("/uav/flightgoggles_ros_bridge/timestampfile_path", timestampFilePath_)) {
        ROS_INFO( "Did not get argument for timestampfile_path. Will render all frames" );
    } else {
        uint64_t stamp_us;
        std::ifstream infile (timestampFilePath_);
        while (infile >> stamp_us) {
            timestampsToRender_.push_back(stamp_us);
        }
    }

    


    // Load params
    populateRenderSettings();

    imageTriggerDebugPublisher_ = ns_.advertise<std_msgs::Empty>("/uav/camera/debug/render_trigger", 60);

    // Collision publisher
    collisionPub_ = ns_.advertise<std_msgs::Empty>("/uav/collision", 1);

    lidarPub_ = ns_.advertise<sensor_msgs::Range>("/uav/sensors/downward_laser_rangefinder", 1);

    // IR Marker publisher
    irMarkerPub_ = ns_.advertise<flightgoggles::IRMarkerArray>("/uav/camera/left/ir_beacons", 1);


    // Subscribe to TF messages
    tfSubscriber_ = ns_.subscribe("/tf", 1, &ROSClient::tfCallback, this);

    // Publish the estimated latency
    fpsPublisher_ = ns_.advertise<std_msgs::Float32>("/uav/camera/debug/fps", 1);  

    timeOfLastRender_ = ros::Time(0);

    }

sensor_msgs::CameraInfo ROSClient::GetCameraInfo(std::string &camera_name) {
    
    sensor_msgs::CameraInfo cameraInfo; 
    cameraInfo.width = flightGoggles.state.camWidth;
    cameraInfo.height = flightGoggles.state.camHeight;
    cameraInfo.distortion_model = "plumb_bob";
    float f = (cameraInfo.height / 2.0) / tan((M_PI * (flightGoggles.state.camFOV / 180.0)) / 2.0);
    float cx = cameraInfo.width / 2.0;
    float cy = cameraInfo.height / 2.0;
    // Check if camera is the right camera in a stereo pair. Use 0 by default.
    double baseline_translation = 0;
    ros::param::param<double>("/sensors/camera/"+camera_name+"/baseline_translation", baseline_translation,0); 
    float tx = -f * baseline_translation ; // -fx' * B
    float ty = 0.0;
    cameraInfo.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    cameraInfo.K = {f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0};
    cameraInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cameraInfo.P = {f, 0.0, cx, tx, 0.0, f, cy, ty, 0.0, 0.0, 1.0,
                            0.0};
    return cameraInfo;
}

unity_outgoing::Camera_t ROSClient::GetCameraRenderInfo(std::string &camera_name) {
  unity_outgoing::Camera_t camera;
  ros::param::param<std::string>("/sensors/camera/"+camera_name+"/tf", camera.ID, camera_name);
  ros::param::param<int>("/sensors/camera/"+camera_name+"/outputShaderType", camera.outputShaderType, -1);
  ros::param::param<bool>("/sensors/camera/"+camera_name+"/hasCollisionCheck", camera.hasCollisionCheck, false);
  ros::param::param<bool>("/sensors/camera/"+camera_name+"/doesLandmarkVisCheck", camera.doesLandmarkVisCheck, false);
  return camera;
}


void ROSClient::populateRenderSettings() {
    // Scene/Render settings
    flightGoggles.state.sceneFilename = sceneFilename_;
    flightGoggles.state.camWidth = imageWidth_;
    flightGoggles.state.camHeight = imageHeight_;
    flightGoggles.state.obstaclePerturbationFile = obstaclePerturbationFile_;

    for (auto camera_name : cameraNameList_){
        // Add camera settings to list.
        cameraInfoList_.push_back(GetCameraInfo(camera_name));
        // cameraMetadataList_.push_back(GetCameraRenderInfo(camera_name));
        unity_outgoing::Camera_t cam_obj = GetCameraRenderInfo(camera_name);
        imagePubList_.push_back(it_.advertiseCamera("/uav/camera/"+camera_name+(((cam_obj.outputShaderType != 2) && (cam_obj.outputShaderType != 5))? "/image_rect_color" : "/grayscale"), 60));
        flightGoggles.state.cameras.push_back(cam_obj);
        cameraTFList_.push_back(cam_obj.ID);
    }

}


// Subscribe to all TF messages.
// To make sure that all all TFs are populated in the tree,
// this callback buffers requests by 1 frame.
void ROSClient::tfCallback(tf2_msgs::TFMessage::Ptr msg){
    //geometry_msgs::TransformStamped world_to_uav;
    bool found_transform = false;
    usleep(1000);

    // Check if TF message is for world->uav/imu
    // This is output by dynamics node.
    ros::Time tfTimestamp;
    for (auto transform : msg->transforms){
        if (transform.child_frame_id == bodyFrame_){
            //world_to_uav = transform;
            found_transform = true;
            tfTimestamp = transform.header.stamp;
        }
    }

    // Skip if do not have transform
    if (!found_transform) return;

    // Only send a render request if necessary to achieve the required framerate
    bool renderFrameBasedOnFramerate = (tfTimestamp >= timeOfLastRender_ + ros::Duration(1.0f/(framerate_ + 1e-9)));
    bool timestampAppearsInFile = std::binary_search(timestampsToRender_.begin(), timestampsToRender_.end(), uint64_t(tfTimestamp.toNSec()*1e-3));
    bool shouldUseFileTimes = timestampsToRender_.size();

    if ( (shouldUseFileTimes && timestampAppearsInFile) || (!shouldUseFileTimes && renderFrameBasedOnFramerate) ){
        // Buffer render requests by 1 frame to allow for TFs to arrive on time.
        long renderTimestamp = timeOfLastRender_.toNSec();
        timeOfLastRender_ = tfTimestamp;
        flightGoggles.state.ntime = renderTimestamp;

        // Send request for all cameras.
        for (int i =0; i < cameraNameList_.size(); i++) {
            std::string camera_name = cameraNameList_[i];
            std::string tf_name = cameraTFList_[i];
            geometry_msgs::TransformStamped camTransform;

            // Get transform (synced).
            ros::Time captureTime;
            captureTime.fromNSec(flightGoggles.state.ntime);
            try{
                camTransform = tfBuffer_.lookupTransform(worldFrame_, "uav/camera/"+tf_name+"/ned", captureTime);
            } catch (tf2::TransformException &ex) {
                ROS_WARN("Could NOT find transform for /uav/camera/%s/ned: %s", tf_name.c_str(), ex.what());
            }

            Transform3 camPose = tf2::transformToEigen(camTransform);
            flightGoggles.setCameraPoseUsingROSCoordinates(camPose, i);
            
            // Check that render timestamps are sync'd
            if (i>0 && flightGoggles.state.ntime != camTransform.header.stamp.toNSec()){
                ROS_ERROR("Warning: Camera TF timestamps do not match!"); 
                std::cout << flightGoggles.state.ntime << " " << camTransform.header.stamp.toNSec() << std::endl;
                // ROS_ERROR(camTransform.header.stamp.toNSec());               
            }

        }
       
        // Find list of all obstacles in environment ("flightgoggles_obstacle/*")
        for (auto obstacleTFName : obstacleTFList_){
        
            // Get TF for obstacle
            geometry_msgs::TransformStamped obstacleTF;
            bool foundTransform = false;
            try{
                obstacleTF = tfBuffer_.lookupTransform(worldFrame_, obstacleTFName, ros::Time(0));
                foundTransform = true;
            } catch (tf2::TransformException &ex) {
                ROS_WARN("Could NOT find transform for obstacle: %s", ex.what());
            }

            // @TODO: Populate obstacle structure w/ transform. Setter function should check that prefab name has already been populated.  
            if (foundTransform) {
                Transform3 objectPose = tf2::transformToEigen(obstacleTF);
                flightGoggles.setObjectPoseUsingROSCoordinates(objectPose, obstacleTFName);
            }
        }
        // request render
        flightGoggles.requestRender();
        // Send shutter trigger
        std_msgs::Empty msg;
    	imageTriggerDebugPublisher_.publish(msg);

    }
} 

// New thread for republishing received images
void imageConsumer(ROSClient *self){
    while (true){
        // Wait for render result (blocking).
        unity_incoming::RenderOutput_t renderOutput = self->flightGoggles.handleImageResponse();

        // Extract image timestamp
        ros::Time imageTimestamp;
        imageTimestamp = imageTimestamp.fromNSec(renderOutput.renderMetadata.ntime);

        // Calculate average FPS every second.
	    std_msgs::Float32 fps_;
	    if (self->timeSinceLastMeasure_.toSec() != 0) {
	    double t = (ros::WallTime::now() - self->timeSinceLastMeasure_).toSec();
	        if (t > 1) {
	            fps_.data = self->frameCount_ / t;
	            self->fpsPublisher_.publish(fps_);
                self->timeSinceLastMeasure_ = ros::WallTime::now();
                self->frameCount_ = 0;
            }
        } else {
            self->timeSinceLastMeasure_ = ros::WallTime::now();
        }

        // Loop through and republish all images
        for (int i = 0; i < renderOutput.images.size(); i++){
            // Convert OpenCV image to image message
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), (renderOutput.renderMetadata.channels[i] == 3) ? "bgr8" : "8UC1", renderOutput.images[0]).toImageMsg();
            msg->header.stamp = imageTimestamp;
            msg->header.frame_id = "/uav/camera/"+self->cameraTFList_[i];
            // Add Camera info message for camera
            sensor_msgs::CameraInfoPtr cameraInfoMsgCopy(new sensor_msgs::CameraInfo(self->cameraInfoList_[i]));
            cameraInfoMsgCopy->header.frame_id = "/uav/camera/"+self->cameraTFList_[i];
        cameraInfoMsgCopy->header.stamp = imageTimestamp;
            self->imagePubList_[i].publish(msg, cameraInfoMsgCopy);
        }

        // Check for camera collision
        if (renderOutput.renderMetadata.hasCameraCollision){
            std_msgs::Empty msg;
            self->collisionPub_.publish(msg);
        }

        // Publish lidar range finder message
        sensor_msgs::Range lidarReturnMsg;
        lidarReturnMsg.header.stamp = imageTimestamp;
        lidarReturnMsg.header.frame_id = "/uav/imu";
        lidarReturnMsg.radiation_type = lidarReturnMsg.INFRARED;
        lidarReturnMsg.field_of_view = 0;
        lidarReturnMsg.min_range = -1.0f*self->lidarMaxRange_;
        lidarReturnMsg.max_range = 0;
        // Add noise to lidar reading if reading is valid.
        // Make reading negative since the distance is in the -Z direction in '/uav/imu' frame.
        lidarReturnMsg.range = static_cast<float>(-1.0f * (renderOutput.renderMetadata.lidarReturn + sqrt(self->lidarVariance_) * self->standardNormalDistribution_(self->randomNumberGenerator_)));

        self->lidarPub_.publish(lidarReturnMsg);

        // Publish IR marker locations visible from FlightGoggles.
        flightgoggles::IRMarkerArray visiblePoints;
        visiblePoints.header.stamp = imageTimestamp;

        for (const unity_incoming::Landmark_t landmark : renderOutput.renderMetadata.landmarksInView){

            // Create marker msg
            flightgoggles::IRMarker marker;
            // Get name of gate and marker
            std::vector<std::string> IDTokens;
            boost::split(IDTokens, landmark.ID, boost::is_any_of("_"));

            marker.landmarkID.data = IDTokens.at(0);
            marker.markerID.data = IDTokens.at(1);

            // Get location and convert to pixel space with origin at upper left corner (OpenCV convention)
            marker.x = landmark.position.at(0) * renderOutput.renderMetadata.camWidth;
            // Flip pixel convention to match openCV. Origin is upper left after conversion.
            marker.y = (1.0f-landmark.position.at(1)) * renderOutput.renderMetadata.camHeight;
            // marker.z = landmark.position.at(2);
            marker.z = 0; // Do not publish distance.

            visiblePoints.markers.push_back(marker);
        }

        // Publish marker message
        self->irMarkerPub_.publish(visiblePoints);

        // Display result
        if (SHOW_DEBUG_IMAGE_FEED){
            cv::imshow("Debug RGB", renderOutput.images[0]);
            //cv::imshow("Debug D", renderOutput.images[1]);
            cv::waitKey(1);
        }

        self->frameCount_ ++;

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "flightgoggles_ros_bridge");

    ros::NodeHandle ns;
    ros::NodeHandle ns_private("flightgoggles_ros_bridge");
    
    // Create client
    ROSClient client(ns, ns_private);

    // Fork a sample image consumer thread
    std::thread imageConsumerThread(imageConsumer, &client);


    // Spin
    ros::spin();

    return 0;
}
