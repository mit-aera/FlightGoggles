#ifndef UNITYMESSAGESPEC_H
#define UNITYMESSAGESPEC_H
/**
 * @file   jsonMessageSpec.hpp
 * @author Winter Guerra
 * @brief  Defines the json structure of messages going to and from Unity.
 */

// Message/state struct definition

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "json.hpp"
using json = nlohmann::json;

namespace unity_outgoing
{

struct Camera_t
{
  std::string ID;
  std::string TF;
  // Position and rotation use Unity left-handed coordinates.
  // Z North, X East, Y up.
  // E.G. East, Up, North.
  std::vector<double> position;
  std::vector<double> rotation;
  // Metadata
  // enum CameraShader {
  //   RGB = -1,
  //   InstanceID = 0,
  //   SemanticID = 1,
  //   DepthCompressed = 2,
  //   DepthMultiChannel = 3,
  //   SurfaceNormals =4
  //   grayscale=5
  //   
  //   }
  int outputShaderType = -1;

  // Motion blur, can takes values from [0,1], where 1.0 means that the shutter is open across the entire exposure time.
  float motionBlurPercent = 0.0;
  
  // Should this camera collision check or check for visibility?
  bool hasCollisionCheck = true;
  bool doesLandmarkVisCheck = false;
};

// Window class for decoding the ZMQ messages.
struct Object_t
{
  std::string ID;
  std::string prefabID;
  std::vector<double> position;
  std::vector<double> rotation;
  // Metadata
  std::vector<double> size;
};



struct StateMessage_t
{
  // Scene/Render settings
  bool sceneIsInternal = true;
  // Scene choices in v1.4.1
  // std::string sceneFilename = "Hazelwood_Loft_Full_Night";
  // std::string sceneFilename = "Hazelwood_Loft_Full_Day";
  // std::string sceneFilename = "Butterfly_World";
  // std::string sceneFilename = "NYC_Subway";
  // std::string sceneFilename = "Museum_Day";
  std::string sceneFilename = "Museum_Day_Small";
  std::string obstaclePerturbationFile = "";


  // Frame Metadata
  int64_t ntime;
  int camWidth = 1024;
  int camHeight = 768;
  float camFOV = 70.0f;
  double camDepthScale = 10.0; // 0.xx corresponds to xx cm resolution

  // Object state update
  std::vector<Camera_t> cameras;
  std::vector<Object_t> objects;
//  std::vector<Landmark_t> landmarks;
};

// Json constructors

// StateMessage_t
inline void to_json(json &j, const StateMessage_t &o)
{
  j = json{// Initializers
//           {"maxFramerate", o.maxFramerate},
           {"sceneIsInternal", o.sceneIsInternal},
           {"sceneFilename", o.sceneFilename},
           {"obstaclePerturbationFile", o.obstaclePerturbationFile},

           // Frame Metadata
           {"ntime", o.ntime},
           {"camWidth", o.camWidth},
           {"camHeight", o.camHeight},
           {"camFOV", o.camFOV},
           {"camDepthScale", o.camDepthScale},
           // Object state update
           {"cameras", o.cameras},
           {"objects", o.objects}
  };
}

// Camera_t
inline void to_json(json &j, const Camera_t &o)
{
  j = json{{"ID", o.ID},
           {"position", o.position},
           {"rotation", o.rotation},
           {"outputShaderType", o.outputShaderType},
           {"motionBlurPercent", o.motionBlurPercent},
           {"hasCollisionCheck", o.hasCollisionCheck},
           {"doesLandmarkVisCheck", o.doesLandmarkVisCheck}
  };
}

// Object_t
inline void to_json(json &j, const Object_t &o)
  {
    j = json{
      {"ID", o.ID},
      {"prefabID", o.prefabID},
      {"position", o.position},
      {"rotation", o.rotation},
      {"size", o.size}
    };
  }

}

// Struct for returning metadata from Unity.
namespace unity_incoming
{

    // Points in the world that should be checked for visibility from some camera.
    struct Landmark_t
    {
        std::string ID;
        std::vector<double> position;
    };

struct RenderMetadata_t
{
  // Metadata
  int64_t ntime;
  int camWidth;
  int camHeight;
  double camDepthScale;
  // Object state update
  std::vector<std::string> cameraIDs;
  std::vector<int> channels;

  // Status update from collision detectors and raycasters.
  bool hasCameraCollision = false;
  std::vector<Landmark_t> landmarksInView;
  float lidarReturn = 100.0f;
};

// Json Parsers

// Landmark_t
    inline void from_json(const json &j, Landmark_t &o)
    {
      o.ID = j.at("ID").get<std::string>();
      o.position = j.at("position").get<std::vector<double>>();
    }

// RenderMetadata_t
inline void from_json(const json &j, RenderMetadata_t &o)
{
  // Get timestamp
  if (j.find("ntime") != j.end() ){
    o.ntime = j.at("ntime").get<int64_t>();  
  } else {
    // Backwards compatibility for FlightGoggles API <= v1.7.0
    o.ntime = j.at("utime").get<int64_t>();
  }
   
  o.camWidth = j.at("camWidth").get<int>();
  o.camHeight = j.at("camHeight").get<int>();
  o.camDepthScale = j.at("camDepthScale").get<double>();
  o.cameraIDs = j.at("cameraIDs").get<std::vector<std::string>>();
  o.channels = j.at("channels").get<std::vector<int>>();
  
  // Backwards compatibility for FlightGoggles API <= v1.7.0  
  if (j.find("hasCameraCollision") != j.end() ){
    o.hasCameraCollision = j.at("hasCameraCollision").get<bool>();  
  }
  if (j.find("landmarksInView") != j.end()){
    o.landmarksInView = j.at("landmarksInView").get<std::vector<Landmark_t>>();
  }
  if (j.find("lidarReturn") != j.end()){
    o.lidarReturn = j.at("lidarReturn").get<float>();
  }

}

// Struct for outputting parsed received messages to handler functions
struct RenderOutput_t
{
  RenderMetadata_t renderMetadata;
  std::vector<cv::Mat> images;
};
}

#endif
