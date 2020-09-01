#! /usr/bin/python
import rospy
import cv2, csv
from cv_bridge import CvBridge
import numpy as np
import std_msgs, geometry_msgs, nav_msgs, sensor_msgs, tf2_geometry_msgs
import tf2_ros, tf
import os

class transforms:

    def get_inverse_tf(self, T):
        inv = np.identity(4)

        R = T[0:3,0:3]
        t = T[0:3,3]
        inv[0:3,0:3] = R.transpose()
        inv[0:3,3] = np.matmul(-R.transpose(), t)
        return inv

    def get_odom_tf(self, odom):
        init_x = 0.0
        init_y = 0.0
        init_z = 0.0
        init_pos_set = False
        T = self.get_odom_tf_ex(odom, init_x, init_y, init_z, init_pos_set, False)
        return T

    def get_odom_tf_ex(self, odom, init_x, init_y, init_z, init_pos_set, zero_odom):
        Toi = np.zeros(shape=(4,4))
        if zero_odom:
            if init_pos_set == False:
                init_x = odom.pose.pose.position.x
                init_y = odom.pose.pose.position.y
                init_z = odom.pose.pose.position.z
                init_pos_set = True
        EPS = 1e-15
        x = odom.pose.pose.position.x - init_x
        y = odom.pose.pose.position.y - init_y
        z = odom.pose.pose.position.z - init_z
        qx = odom.pose.pose.orientation.x
        qy = odom.pose.pose.orientation.y
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w
        p = np.matrix([[x],[y],[z]])
        qbar = np.matrix([[qx],[qy],[qz],[qw]])
        R = np.identity(3)
        if np.matmul(qbar.transpose(),qbar) > EPS:
            epsilon = np.matrix([[qx],[qy],[qz]])
            eta = qw
            epsilon_cross = np.matrix([[0, -epsilon[2], epsilon[1]], [epsilon[2], 0, -epsilon[0]], [-epsilon[1], epsilon[0], 0]])
            I = np.identity(3)
            R = (eta**2 - np.matmul(epsilon.transpose(), epsilon)[0,0])*I + np.subtract(2*(epsilon*epsilon.transpose()), 2*eta*epsilon_cross)
        Toi[0:3,0:3] = R.transpose()
        Toi[0:3,3] = p.reshape(3)
        Toi[3,0:3] = np.zeros(shape=(1,3))
        Toi[3,3] = 1.0
        return Toi

    def convert_to_eigen(self, tf):
        q_ros = tf.transform.rotation
        p_ros = tf.transform.translation
        T = np.zeros(shape=(4,4))
        p = np.matrix([[p_ros.x], [p_ros.y], [p_ros.z]])
        epsilon = np.matrix([[q_ros.x], [q_ros.y], [q_ros.z]])
        eta = q_ros.w
        epsilon_cross = np.matrix([[0, -epsilon[2], epsilon[1]], [epsilon[2], 0, -epsilon[0]], [-epsilon[1], epsilon[0], 0]])
        I = np.identity(3)
        R = (eta**2 - np.matmul(epsilon.transpose(), epsilon)[0,0])*I + np.subtract(2*(epsilon*epsilon.transpose()), 2*eta*epsilon_cross)
        p = -1 * R * p
        T[0:3,0:3] = R.transpose()
        T[0:3,3] = p.reshape(3)
        T[3,0:3] = np.zeros(shape=(1,3))
        T[3,3] = 1.0
        return T

    def get_transform(self, src_frame, tgt_frame):
        got_tf = False
        timeout = 0.025
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        while (not got_tf and not rospy.is_shutdown()):
            try:
                tf_temp = tfBuffer.lookup_transform(src_frame, tgt_frame, rospy.Time(), rospy.Duration(timeout))
                got_tf = True
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                got_tf = False
                rospy.logwarn_throttle(1, "Transform from " + src_frame + " to " + tgt_frame + " was not found! Is TF tree and clock being published?")
            if got_tf:
                T = self.convert_to_eigen(tf_temp)
            else:
                T = np.zeros(shape=(4,4))
        return T

class TriangulationNode:
    def set_camera_matrix(self, CAM):
        P_ = CAM
        return True

    def set_node_name(self):
        global node_name
        node_name = rospy.get_name()
        return True

    def initialize_transforms(self):
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        global Tic
        x = transforms()
        Tic = x.get_transform("mono_link", "imu_link")
        return True

    def get_ros_parameters(self):
        global camera_frame
        global image_rate
        global triang_file
        global folder
        global collect_image
        camera_frame = rospy.get_param(node_name + "/camera_frame")
        image_rate = rospy.get_param(node_name + "/image_rate")
        triang_file = rospy.get_param(node_name + "/triang_file")
        folder = rospy.get_param(node_name + "/folder")
        collect_image = rospy.get_param(node_name + "/collect_image")
        return True

def callback(img, odom):
    bridge = CvBridge()
    start = rospy.rostime.Time.now()
    node_name = rospy.get_name()
    img_save = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
    data = open(triang_file, 'a')
    mycsv = csv.writer(data)
    if collect_image == 1:
        rospy.Rate(image_rate)
        filetime = rospy.rostime.Time.now()
        filename = folder + str(filetime) + ".png"
        cv2.imwrite(filename, img_save)
        transform = transforms()
        Toi = transform.get_odom_tf(odom)
        Tio = transform.get_inverse_tf(Toi)
        Tic = transform.get_transform(camera_frame, "imu_link")
        E = np.zeros(shape =(4,4))
        E = transform.get_inverse_tf(Toi * Tic)
        P = rospy.get_param(node_name + "/P")
        P = np.matrix(P)
        P = P.reshape(4,4)
        P_tilda = np.matmul(P, E)
        P_tilda = np.asarray(P_tilda).reshape(-1)
        Tio = np.asarray(Tio).reshape(-1)
        Pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
        row = [filename, P_tilda, Tio, Pose]
        mycsv.writerow(row)
        data.close()
    stop = rospy.rostime.Time.now()
    elapsed = stop - start
    rospy.logdebug("[OBJ] TRIANGULATION TIME: " + str(elapsed))
    return True
