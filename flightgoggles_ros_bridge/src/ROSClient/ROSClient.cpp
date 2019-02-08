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

    // Wait for static transforms between imu/cameras.
    try{
        imu_T_Camera_ = tfBuffer_.lookupTransform( "uav/camera/center", "uav/imu", ros::Time(0),
                                                    ros::Duration(10.0));
//        imu_T_Camera_ = tfBuffer_.lookupTransform(  "uav/imu", "uav/camera/center", ros::Time(0),
//                                                   ros::Duration(10.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT find required uav to camera transform: %s", ex.what());
        exit(1);
    }

    // Load params
    populateRenderSettings();

    // init image publisher
    imagePubLeft_ = it_.advertiseCamera("/uav/camera/left/image_rect_color", 1);
    //imagePubRight_ = it_.advertiseCamera("/uav/camera/right/image_rect_color", 1);

    // Collision publisher
    collisionPub_ = ns_.advertise<std_msgs::Empty>("/uav/collision", 1);

    // IR Marker publisher
    irMarkerPub_ = ns_.advertise<flightgoggles::IRMarkerArray>("/uav/camera/left/ir_beacons", 1);


    // Subscribe to TF messages
    tfSubscriber_ = ns_.subscribe("/tf", 1, &ROSClient::tfCallback, this);

    // Publish the estimated latency
    fpsPublisher_ = ns_.advertise<std_msgs::Float32>("/uav/camera/debug/fps", 1);    

    // Subscribe to IR beacon locations
//    irSubscriber_ = ns_.subscribe("/challenge/ir_BeaconsGroundTruth", 1, &ROSClient::irBeaconPointcloudCallback, this);
}

void ROSClient::populateRenderSettings(){
  // Scene/Render settings
  /*
  Available scenes:
  sceneFilename = "Butterfly_World";
  sceneFilename = "FPS_Warehouse_Day";
  sceneFilename = "FPS_Warehouse_Night";
  sceneFilename = "Hazelwood_Loft_Full_Day";
  sceneFilename = "Hazelwood_Loft_Full_Night";

   // NEW for FlightGoggles v2.x.x
   flightGoggles.state.sceneFilename = "Abandoned_Factory_Morning";
   flightGoggles.state.sceneFilename = "Abandoned_Factory_Sunset";
   */

  flightGoggles.state.sceneFilename = "Abandoned_Factory_Morning";


    // Prepopulate metadata of cameras
  unity_outgoing::Camera_t cam_RGB;
  cam_RGB.ID = "Camera_RGB";
  cam_RGB.channels = 3;
  cam_RGB.isDepth = false;
  cam_RGB.outputIndex = 0;
  cam_RGB.hasCollisionCheck = true;
  cam_RGB.doesLandmarkVisCheck = true;

  // Add cameras to persistent state
  flightGoggles.state.cameras.push_back(cam_RGB);
  
  // set up the CameraInfo struct
  cameraInfo = {};
  cameraInfo.width = flightGoggles.state.camWidth;
  cameraInfo.height = flightGoggles.state.camHeight;
  cameraInfo.distortion_model = "plum_bob";
  float f = (cameraInfo.height/2.0)/tan((M_PI*(flightGoggles.state.camFOV/180.0))/2.0);
  float cx = cameraInfo.width/2.0;
  float cy = cameraInfo.height/2.0;
  cameraInfo.D = {0.0, 0.0, 0.0, 0.0, 0.0};
  cameraInfo.K = {f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0};
  cameraInfo.R = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  cameraInfo.P = {f, 0.0, cx, 0, 0.0, f, cy, 0, 0.0, 0.0, 1.0, 0.0}; // elements Tx and Ty should be different for the second camera in a stereo pair. Here they are 0.
}

// Subscribe to all TF messages to avoid lag.
void ROSClient::tfCallback(tf2_msgs::TFMessage::Ptr msg){
    geometry_msgs::TransformStamped world_to_uav;
    bool found_transform = false;

    // Check if TF message is for world->uav/imu
    for (auto transform : msg->transforms){
        if (transform.child_frame_id == "uav/imu"){
            world_to_uav = transform;
            found_transform = true;
        }
    }

    // Skip if do not have transform
    if (!found_transform) return;

    // Skip every other transform to get an update rate of 60hz
    if (numSimulationStepsSinceLastRender_ >= numSimulationStepsBeforeRenderRequest_){

        Transform3 imu_pose_eigen = tf2::transformToEigen(world_to_uav.transform);
//        Transform3 cam_pose_eigen;
        // Apply drone to camera transform
//        tf2::doTransform(imu_pose_eigen, cam_pose_eigen, imu_T_Camera_);

//        std::cout << imu_pose_eigen.matrix() << std::endl;

        // Populate status message with new pose
        flightGoggles.setCameraPoseUsingROSCoordinates(imu_pose_eigen, 0);
//    flightGoggles.setCameraPoseUsingROSCoordinates(cam_pose_eigen, 1);

        // Update timestamp of state message (needed to force FlightGoggles to rerender scene)
        flightGoggles.state.ntime = world_to_uav.header.stamp.toNSec();
        // request render
        flightGoggles.requestRender();

        numSimulationStepsSinceLastRender_ = 0;

    } else {
        numSimulationStepsSinceLastRender_++;
    }

}

//void ROSClient::irBeaconPointcloudCallback(sensor_msgs::PointCloud2::Ptr msg) {
//    irBeaconGroundTruth_ = msg;
//}

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

        // Convert OpenCV image to image message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", renderOutput.images[0]).toImageMsg();
        msg->header.stamp = imageTimestamp;
        // Add Camera info message for camera
        sensor_msgs::CameraInfoPtr cameraInfoMsgCopy(new sensor_msgs::CameraInfo(self->cameraInfo));
        cameraInfoMsgCopy->header.stamp = imageTimestamp;
	    self->imagePubLeft_.publish(msg, cameraInfoMsgCopy);

        // Check for camera collision
        if (renderOutput.renderMetadata.hasCameraCollision){
            std_msgs::Empty msg;
            self->collisionPub_.publish(msg);
        }

        // Publish IR marker locations visible from FlightGoggles.
        flightgoggles::IRMarkerArray visiblePoints;
        visiblePoints.header.stamp = imageTimestamp;

        for (const unity_incoming::Landmark_t landmark : renderOutput.renderMetadata.landmarksInView){
//            std::cout << landmark.ID << std::endl;

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

    // Start render requests

//    callback tfCallback = boost::bind(&ROSClient::renderLoopTimerCallback, &client, _1);
//    ros::Subscriber sub = ns.subscribe("/tf", 1, tfSubscriber);

    // Spin
    ros::spin();

    return 0;
}
