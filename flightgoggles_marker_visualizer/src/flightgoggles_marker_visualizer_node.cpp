/** @file flightgoggles_marker_visualizer_node.cpp
 * @author Sebastian Quilter
 * @brief Visualizes the IR markers on the image
 *
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <flightgoggles/IRMarkerArray.h>

using namespace sensor_msgs;
using namespace message_filters;

image_transport::Publisher image_pub;

void callback(const ImageConstPtr& image, const flightgoggles::IRMarkerArrayConstPtr& marker_array)
{
  cv_bridge::CvImagePtr imagePtr;
  imagePtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8); 

  cv::Scalar color(0,0,255);

  for (flightgoggles::IRMarker marker : marker_array->markers){
    cv::circle(imagePtr->image, cv::Point(marker.x, marker.y), 5, color, -1);
  }


  image_pub.publish(imagePtr->toImageMsg());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bounding_box_node");

  ros::NodeHandle nh;

  std::string image_topic_name ("/uav/camera/left/image_rect_color");
  std::string bounding_box_topic_name ("/uav/camera/left/ir_beacons");

  message_filters::Subscriber<Image> image_sub(nh, image_topic_name, 1);
  message_filters::Subscriber<flightgoggles::IRMarkerArray> marker_sub(nh, bounding_box_topic_name, 1);
  TimeSynchronizer<Image, flightgoggles::IRMarkerArray> sync(image_sub, marker_sub, 50);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  image_transport::ImageTransport it(nh);
  image_pub = it.advertise("/bounding_box_camera/RGB", 1);

  ros::spin();

  return 0;
}

