#!/usr/bin/env python2
import rospy, time, os, shutil, subprocess, sys
from sensor_msgs.msg import CameraInfo

'''
Winter Guerra <winterg@mit.edu>, June 11th, 2019
Distributed under MIT License.

This node dynamically spawns camera saver nodes depending on what cameras have been defined in FlightGoggles.

'''

if __name__ == '__main__':
    rospy.init_node('flightgoggles_image_saver_orchestrator', anonymous=True)

    # Get image output location
    outputFolder = rospy.get_param('~output_folder')

    # Get list of cameras
    cameraNameList = rospy.get_param('/sensors/camera/camera_list', default=[])

    # ensure that parent directory exists
    #try:
    #    shutil.rmtree(outputFolder)
    #except:
    #    pass
    
    #os.mkdir(outputFolder)
        

    # Get image types for each camera
    for cam_name in cameraNameList:
        
        cameraOutputType = rospy.get_param('/sensors/camera/{}/outputShaderType'.format(cam_name), default=-1)
        outputIsGrayscale = ((cameraOutputType == 2) or (cameraOutputType == 5))
        cameraID = rospy.get_param('/sensors/camera/{}/ID'.format(cam_name), default="")
        cameraTopicName = "/uav/camera/{}/{}".format(cam_name, "image_rect_color" if (not outputIsGrayscale) else "grayscale")
        cameraOutputFolder = os.path.join(outputFolder, cameraID)
        cameraEncoding = "8UC1" if (outputIsGrayscale) else "8UC3"
        
        # Ensure that output folder exists and can be written to.
        try:
            shutil.rmtree(cameraOutputFolder)
        except:
            pass
        os.mkdir(cameraOutputFolder)
        
        command = "rosrun image_view image_saver image:={} _output_folder:={}/".format(cameraTopicName, cameraOutputFolder )#, "bgr8") #cameraEncoding) 
        print command

        # Spawn an image saver node
        process = subprocess.Popen(command, shell=True) #, stdout="/dev/null")

        # If in debug mode, spawn viewer windows.
        #command = "rosrun image_view image_view image:={} _autosize:=true".format(cameraTopicName)
        #process = subprocess.Popen(command, shell=True)
        

        # process.wait()   

    
    rospy.spin()
