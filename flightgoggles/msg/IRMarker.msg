# ROS Message that contains pixel-space IR marker locations and labels.

# ID of the landmark (e.g. gate) that the IR marker is attached to
std_msgs/String landmarkID
# ID of individual marker.
std_msgs/String markerID

float32 x
float32 y
# Z is the distance from the camera in meters.
float32 z