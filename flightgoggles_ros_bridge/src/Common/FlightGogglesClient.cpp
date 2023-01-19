/**
 * @file   FlightGogglesClient.cpp
 * @author Winter Guerra
 * @brief  Node that outputs the expected pose of cameras based on motion capture
 * data to Unity for FlightGoggles operation.
 */

#include "FlightGogglesClient.hpp"

#define DEBUG false



/**
 * Constructor
 */
FlightGogglesClient::FlightGogglesClient()
{
    initializeConnections();
}

/**
 * initializeConnections initializeConnections with Unity
 */
void FlightGogglesClient::initializeConnections()
{
    std::cout << "Initializing ZMQ connections..." << std::endl;
    // create and bind a upload_socket
    //upload_socket.set(zmqpp::socket_option::send_high_water_mark, 6);
    upload_socket.bind(client_address + ":" + upload_port);
    // create and bind a download_socket
    //download_socket.set(zmqpp::socket_option::receive_high_water_mark, 6);
//    download_socket.set(zmqpp::socket_option::receive_buffer_size, 1024*768*3*6);
    download_socket.bind(client_address + ":" + download_port);
    download_socket.subscribe("");
    std::cout << "Done!" << std::endl;
}

/**
 * setCameraPoseUsingROSCoordinates accepts camera pose in ros frame. Expects NED world frame and NED camera frame.
 * @param ros_pose Pose of the camera
 * @param cam_index Index of the camera
 */
void FlightGogglesClient::setCameraPoseUsingROSCoordinates(Transform3 ros_pose, int cam_index, double scene_scale/*=1.0*/) {
  // To transforms
  //Transform3 NED_pose = convertROSToNEDCoordinates(ros_pose);
  Transform3 unity_pose = convertNEDGlobalPoseToGlobalUnityCoordinates(ros_pose);
//Transform3 unity_pose = convertEDNGlobalPoseToGlobalUnityCoordinates(ros_pose);

  // Extract position and rotation
  std::vector<double> position = {
    unity_pose.translation()[0]*scene_scale,
    unity_pose.translation()[1]*scene_scale,
    unity_pose.translation()[2]*scene_scale,
  };

  Eigen::Matrix3d rotationMatrix = unity_pose.rotation();
  Quaternionx quat(rotationMatrix);

  std::vector<double> rotation = {
    quat.x(),
    quat.y(),
    quat.z(),
    quat.w(),
  };

  // Set camera position and rotation
  state.cameras[cam_index].position = position;
  state.cameras[cam_index].rotation = rotation;
}


void FlightGogglesClient::setObjectPoseUsingROSCoordinates(Transform3 ros_pose, std::string object_name, double scene_scale/*=1.0*/) {
  
  Transform3 unity_pose = convertNEDGlobalPoseToGlobalUnityCoordinates(ros_pose);

  std::cout << "Objects: " << std::endl;
  for (auto o: state.objects) {
      std::cout << o.ID << " " << std::endl;
  }

  // Check if there is a object in the object list that matches the object name.
  auto it = std::find_if(std::begin(state.objects),
                         std::end(state.objects),
                         [object_name]
                         (const unity_outgoing::Object_t& obj) -> bool {return (obj.ID.compare(object_name) == 0);});

  
  std::cout << "Placing object " << object_name << std::endl;
  std::cout << state.objects.size() << std::endl;
  // Create the object if not found
  bool found_object_name = false;
  if (it == std::end(state.objects)){
    unity_outgoing::Object_t object;
    std::string object_type;
    std::cout << "Test" << std::endl;
    // Find the object type:
    const std::regex object_name_regex("flightgoggles_obstacle/(.*)/.*");
    std::smatch object_match;
    if (std::regex_match(object_name, object_match, object_name_regex)) {
      if (object_match.size() == 2){
        object_type = object_match[1];
        found_object_name = true;
        std::cout << "Found object type: " << object_type << std::endl;
      }
    }
        
    if (!found_object_name){
      std::cerr << "Cannot find object name! Are you sure that the object name is of the form 'flightgoggles_obstacle/objPrefabName/idx'" << std::endl;
      return;
    }

    // Create the object
    //unity_outgoing::Object_t object();
    object.ID.assign(object_name);
    object.prefabID.assign(object_type);
    state.objects.push_back(object);
    // Get iterator that points to object
    it = std::prev(std::end(state.objects),1);

  }

  //std::cout << "Trying to do placement calculations" << std::endl;  

  // Extract position and rotation
  std::vector<double> position = {
    unity_pose.translation()[0]*scene_scale,
    unity_pose.translation()[1]*scene_scale,
    unity_pose.translation()[2]*scene_scale,
  };

  std::cout << unity_pose.translation()[0] << " " << unity_pose.translation()[1] << " " << unity_pose.translation()[2] << std::endl;
  if (unity_pose.translation()[0] == 0) {
    std::cout << "Got zero transform!!" << std::endl;
  }

  Eigen::Matrix3d rotationMatrix = unity_pose.rotation();
  Quaternionx quat(rotationMatrix);

    std::vector<double> rotation = {
    quat.x(),
    quat.y(),
    quat.z(),
    quat.w(),
  };

  // Set camera position and rotation
  it->position = position;
  it->rotation = rotation;
  
}


/**
 * This function is called when a new pose has been received.
 * If the pose is good, asks Unity to render another frame by sending a ZMQ
 * message.
*/
bool FlightGogglesClient::requestRender()
{
    // Make sure that we have a pose that is newer than the last rendered pose.
//    if (!(state.ntime > last_uploaded_utime))
//    {
//        // Skip this render frame.
//        return false;
//    }


    // Debug
    // std::cout << "Frame " << std::to_string(state.ntime) << std::endl;

    // Create new message object
    zmqpp::message msg;
    // Add topic header
    msg << "Pose";

    // Update timestamp
    last_uploaded_utime = state.ntime;

    // Create JSON object for status update & append to message.
    json json_msg = state;
    msg << json_msg.dump();

    if (DEBUG) {
      std::cout << "Last message sent: \"";
      std::cout << msg.get<std::string>(0) << std::endl;
      // Print JSON object
      std::cout << json_msg.dump(4) << std::endl;
      std::cout << "===================" << std::endl;
    } 
    
    // Send message 
    upload_socket.send(msg);
    return true;
}

/**
 * handleImageResponse handles the image response from Unity
 * Note: This is a blocking call.
 *
 * @return RenderOutput_t returns the render output with metadata
 */
unity_incoming::RenderOutput_t FlightGogglesClient::handleImageResponse()
{
    // Populate output
    unity_incoming::RenderOutput_t output;

    // Get data from client as fast as possible
    zmqpp::message msg;
    download_socket.receive(msg);

    // Sanity check the packet.
    // if (msg.parts() <= 1)
    // {
    //     return NULL;
    // }
    // Unpack message metadata.
    std::string json_metadata_string = msg.get(0);
    // Parse metadata.
    unity_incoming::RenderMetadata_t renderMetadata = json::parse(json_metadata_string);

    // Log the latency in ms (1,000 microseconds)
    if (!u_packet_latency)
    {
        u_packet_latency = (getTimestamp() - renderMetadata.ntime);
    }
    else
    {
        // avg over last ~10 frames in ms.
        u_packet_latency =
            ((u_packet_latency * (9) + (getTimestamp() - renderMetadata.ntime*1e-3)) / 10.0f);
    }

    ensureBufferIsAllocated(renderMetadata);

    output.images.resize(renderMetadata.cameraIDs.size());

    // For each camera, save the received image.
    //#pragma omp target map(to: msg) map(from: output)
    auto num_threads = renderMetadata.cameraIDs.size();
    const uint8_t stride = 3;
    // #pragma omp parallel
    // #pragma omp for
    for (int i = 0; i < num_threads; i++)
    {

	cv::Mat new_image;

        // Reshape the received image
	if (renderMetadata.channels[i] != 2){
        
          // Get raw image bytes from ZMQ message.
          // WARNING: This is a zero-copy operation that also casts the input to an array of unit8_t.
          // when the message is deleted, this pointer is also dereferenced.
          const uint8_t* imageData;
          msg.get(imageData, i + 1);
          // // ALL images comes as 3-channel RGB images from Unity. Calculate the row length
          // uint32_t bufferRowLength = renderMetadata.camWidth * renderMetadata.channels[i];

          // Pack image into cv::Mat
          new_image = cv::Mat(renderMetadata.camHeight, renderMetadata.camWidth, CV_MAKETYPE(CV_8U, stride));
	  memcpy(new_image.data, imageData, renderMetadata.camWidth * renderMetadata.camHeight * stride );
	} else {
	  // This is a 16UC1 depth image
	  
	  // Get raw image bytes from ZMQ message.
          // WARNING: This is a zero-copy operation that also casts the input to an array of unit8_t.
          // when the message is deleted, this pointer is also dereferenced.
          const uint16_t* imageData;
          msg.get(imageData, i + 1);
          // // ALL images comes as 3-channel RGB images from Unity. Calculate the row length
          // uint32_t bufferRowLength = renderMetadata.camWidth * renderMetadata.channels[i];

          new_image = cv::Mat(renderMetadata.camHeight, renderMetadata.camWidth, CV_MAKETYPE(CV_16U, 1));
	  memcpy(new_image.data, imageData, renderMetadata.camWidth * renderMetadata.camHeight * 2);
 
	}
	    
      // Debug
      //cv::imshow("Debug", new_image);
      //cv::waitKey(0);

        // Tell OpenCv that the input is RGB.
        if (renderMetadata.channels[i]==3){
            if (stride == 3)
              cv::cvtColor(new_image, new_image, cv::COLOR_RGB2BGR);
            if (stride == 4)
              cv::cvtColor(new_image, new_image, cv::COLOR_BGRA2BGR);
        } else if (renderMetadata.channels[i]==1) {
            if (stride == 3)
            cv::cvtColor(new_image, new_image, cv::COLOR_RGB2GRAY);
            if (stride == 4)
            cv::cvtColor(new_image, new_image, cv::COLOR_BGRA2GRAY);
        }

        // Flip image since OpenCV origin is upper left, but Unity's is lower left.
	cv::flip(new_image, new_image, 0);


        // Add image to output vector
        output.images.at(i) = new_image;
    }

    // Add metadata to output
    output.renderMetadata = renderMetadata;

    // Output debug at 1hz
    if (getTimestamp() > last_download_debug_utime + 1e6)
    {
        // Log update FPS
//        std::cout << "Update rate: "
//                  << (num_frames * 1e6) /
//                         (getTimestamp() - last_download_debug_utime)
//                  << " ms_latency: " << u_packet_latency / 1e3 << std::endl;

        last_download_debug_utime = getTimestamp();
        num_frames = 0;
    }
    num_frames++;

    return output;
}
