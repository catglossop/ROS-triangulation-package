#! /usr/bin/python
#Author: Catherine Glossop
import rospy
import std_msgs, nav_msgs
from nav_msgs.msg import Odometry
import message_filters
import csv
import numpy as np
from triangulation_node import *

def main():
    rospy.init_node('triangulation', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    global node_name
    node_name = rospy.get_name()
    #Get parameters
    camera_frame = rospy.get_param(node_name + "/camera_frame")
    camera_topic = rospy.get_param(node_name + "/camera_topic")
    output_append = rospy.get_param(node_name + "/output_append")
    triang_file = rospy.get_param(node_name + "/triang_file")
    folder = rospy.get_param(node_name + "/folder")
    odom = rospy.get_param(node_name + "/odom")
    collect_image = rospy.get_param(node_name + "/collect_image")
    # Subscribers
    sub_img = message_filters.Subscriber(camera_topic, sensor_msgs.msg.Image)
    sub_odom = message_filters.Subscriber(odom, Odometry)
    #Initialize node object
    myNode = TriangulationNode()
    myNode.set_node_name()
    myNode.get_ros_parameters()
    with open(triang_file, 'w') as myFile:
        headings = "Filename; P; Tio; Pose\n"
        writer = csv.writer(myFile)
        writer.writerow(headings)
        myFile.close()
    global P
    P = rospy.get_param(node_name + "/P")
    CAM = np.zeros(shape = (4,4))
    for i in range(0,4):
        for j in range(0,4):
            temp = P[4*i + j]
            CAM[i,j] = temp
    myNode.set_camera_matrix(CAM)
    sync = message_filters.ApproximateTimeSynchronizer([sub_img, sub_odom], 100, 0.1, allow_headerless=True)
    sync.registerCallback(callback)
    rospy.loginfo("AM I IN HERE????")
    rospy.loginfo("[OBJ] Triangulation running")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
