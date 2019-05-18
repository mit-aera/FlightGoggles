#!/usr/bin/env python2
import rospy, time
from sensor_msgs.msg import CameraInfo

'''
Winter Guerra <winterg@mit.edu>, May 14th, 2019
Distributed under MIT License.

This node subscribes to CameraInfo messages and dies when the renderer stops publishing images.
This node can be required by launchfiles that should die after the renderer is finished rendering.
Ex: this node can tell the blackbirdDataset.launch launchfile to exit when a particular trajectory is done rendering.
'''


class FlightGogglesRenderMonitor():

    def __init__(self):
        self.has_started_receiving_data = False
        self.received_data_since_last_check = False
        self.num_seconds_without_data = 0
        self.max_num_seconds_without_data = 5

        pub = rospy.Subscriber('/uav/camera/left/camera_info', CameraInfo, self.cameraInfoCallback)


    def cameraInfoCallback(self, data):
        self.received_data_since_last_check = True
        self.has_started_receiving_data = True
        print "Received data!"


    def spin(self):

        while not rospy.is_shutdown():
            print "loop start!"
            if (self.has_started_receiving_data):
                if (self.received_data_since_last_check):
                    self.num_seconds_without_data = 0
                    self.received_data_since_last_check = False
                    print "yes data"
                else:
                    self.num_seconds_without_data+=1
                    print "no data"

            if (self.num_seconds_without_data > self.max_num_seconds_without_data):
                print "prog end"
                rospy.signal_shutdown("Render monitor has detected a long pause in render output. Shutting down.")
            # Cannot use ROS time because this will hang indefinitely if using simtime and simtime stops.
            time.sleep(1)

if __name__ == '__main__':
    node = FlightGogglesRenderMonitor()
    rospy.init_node('flightgoggles_render_monitor', anonymous=True)
    node.spin()
