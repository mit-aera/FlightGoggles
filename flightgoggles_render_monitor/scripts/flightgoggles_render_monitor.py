#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo

'''
Winter Guerra <winterg@mit.edu>, May 14th, 2019
Distributed under MIT License.

This node subscribes to CameraInfo messages and dies when the renderer stops publishing images.
This node can be required by launchfiles that should die after the renderer is finished rendering.
Ex: this node can tell the blackbirdDataset.launch launchfile to exit when a particular trajectory is done rendering.
'''

has_started_receiving_data = False
num_seconds_without_data = 0
max_num_seconds_without_data = 6


def cameraInfoCallback(data):
    has_started_receiving_data = True
    

def flightGogglesRenderMonitor():
    pub = rospy.Subscriber('/uav/camera/left/camera_info', CameraInfo, cameraInfoCallback)
    rospy.init_node('flightgoggles_render_monitor', anonymous=True)
    rate = rospy.Rate(1) # 1Hz
    
    while not rospy.is_shutdown():
        # Check that 
        rate.sleep()

if __name__ == '__main__':
    flightGogglesRenderMonitor()
