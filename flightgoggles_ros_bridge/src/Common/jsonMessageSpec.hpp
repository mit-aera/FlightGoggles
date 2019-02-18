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
  // Position and rotation use Unity left-handed coordinates.
  // Z North, X East, Y up.
  // E.G. East, Up, North.
  std::vector<double> position;
  std::vector<double> rotation;
  // Metadata
  int channels;
  bool isDepth;
  int outputIndex;
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

  // Frame Metadata
  int64_t ntime;
  int camWidth = 1024;
  int camHeight = 768;
  float camFOV = 70.0f;
  double camDepthScale = 0.20; // 0.xx corresponds to xx cm resolution

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
           {"channels", o.channels},
           {"isDepth", o.isDepth},
           {"outputIndex", o.outputIndex},
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
  bool hasCameraCollision;
  std::vector<Landmark_t> landmarksInView;
  float lidarReturn;
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
  o.ntime = j.at("ntime").get<int64_t>();
  o.camWidth = j.at("camWidth").get<int>();
  o.camHeight = j.at("camHeight").get<int>();
  o.camDepthScale = j.at("camDepthScale").get<double>();
  o.cameraIDs = j.at("cameraIDs").get<std::vector<std::string>>();
  o.channels = j.at("channels").get<std::vector<int>>();
  o.hasCameraCollision = j.at("hasCameraCollision").get<bool>();
  o.landmarksInView = j.at("landmarksInView").get<std::vector<Landmark_t>>();
  o.lidarReturn = j.at("lidarReturn").get<float>();

}

// Struct for outputting parsed received messages to handler functions
struct RenderOutput_t
{
  RenderMetadata_t renderMetadata;
  std::vector<cv::Mat> images;
};
}

#endif
