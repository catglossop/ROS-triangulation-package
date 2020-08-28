#!usr/bin/env python
#Author: Catherine Glossop
import rospy
from std_msgs import String
import message_filters
import csv
import numpy as np
from triangulation_node import *

def main(argc, argv):
    rospy.init_node('triangulation', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #Get parameters
    rospy.get_param(node_name + "/camera_frame",    camera_frame)
    rospy.get_param(node_name + "/camera_topic",    camera_topic)
    rospy.get_param(node_name + "/output_append",   output_append)
    rospy.get_param(node_name + "/triang_file",     triang_file)
    rospy.get_param(node_name + "/folder",          folder)
    rospy.get_param(node_name + "/odom",            odom)
    rospy.get_param(node_name + "/collect_image",    collect_image)
    # Subscribers
    sub_img = message_filters.Subscriber(camera_topic, sensor_msgs.msg.Image)
    sub_odom = message_filters.Subscriber(odom, nav_msgs.msg.Odometry)
    #Initialize node object
    myNode = TriangulationNode()
    myNode.set_node_name()
    myNode.get_ros_parameters()
    myNode.initialize_transforms()
    open(triang_file, 'w') as myFile:
        headings = "Filename; P; Tio; Pose\n"
        writer = csv.writer(myFile)
        writer.write_row(headings)
        myFile.close()
    rospy.get_param(node_name + "/P", P)
    CAM = np.zeros(shape = (4,4))
    for i in range(0,3):
        for j in range(0,3):
            temp = P[4*i*j]
            CAM(i,j) = temp
    myNode.set_camera_matrix(CAM)
    #Synchronize Subscribers
    sync = message_filters.ApproximateTimeSynchronizer([sub_img, sub_odom], 100, 0.1, allow_headerless=True)
    sync.registerCallback(callback)

    ropy.loginfo("[OBJ] Triangulation running")
    rospy.spin()
    return 0
