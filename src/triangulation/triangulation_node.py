import rospy
import cv_bridge
import csv
import numpy as np
from stg_msgs import String
import cv2

class TriangulationNode:
    public:
        def set_camera_matrix(P_):
            P_ = P
            return True
        def set_node_name():
            node_name = rospy.get_name()

        def callback()
