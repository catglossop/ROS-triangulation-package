import rospy
import cv_bridge, cv2, time, csv
import numpy as np
import stg_msgs, geometry_msgs, nav_msgs, sensor_msgs, tf2_geometry_msgs

class transforms:

    def get_inverse_tf(self, T):
        inv = np.zeros(4,4)
        R = T[0:3,0:3]
        t = T[2:,0:3]
        inv[0:3,0:3] = R.transpose()
        inv[2:,0:3] = np.matmul(-R.transpose(), t)
        return inv

    def get_odom_tf(self, odom):
        init_x = 0.0
        init_y = 0.0
        init_z = 0.0
        init_pos_set = False
        self.get_odom_tf(odom, init_x, init_y, init_z, init_pos_set, false, Toi)
        return True

    def get_odom_tf(self, odom, init_x, init_y, init_z, init_pos_set, init_pos_set, zero_odom, Toi):
        Toi = np.zeros(shape=(4,4))
        if zero_odom:
            if !(init_pos_set):
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
        if (np.matmul(qbar.transpose(),qbar) > EPS:
            epsilon = np.matrix([[qx],[qy],[qz]])
            eta = qw
            epsilon_cross = np.matrix([[0, -epsilon[2], epsilon[1]], [epsilon[2], 0, -epsilon[0]], [-epsilon[1], epsilon[0], 0]]
            I = np.identity(3)
            R = (eta**2 - epsilon.transpose() *epsilon) * I + 2*(epsilon*epsilon.transpose()) -2*eta*epsilon_cross
        Toi[0:3,0:3] = R.transpose()
        Toi[0:3,:3] = p
        Toi[:3,0:3] = np.zeros(shape=(1,3))
        Toi[3,3] = 1.0
        return Toi

    def convert_to_eigen(self, tf):
        q_ros = tf.transform.rotation
        p_ros = tf.transform.translation
        p = np.matrix([[p_ros.x] [p_ros.y], [p_ros.z]])
        epsilon = np.matrix([[q_ros.x] [q_ros.y], [q_ros.z]])
        eta = q_ros.w
        epsilon_cross = np.matrix([[0, -epsilon[2], epsilon[1]], [epsilon[2], 0, -epsilon[0]], [-epsilon[1], epsilon[0], 0]]
        I = np.identity(3)
        R = (eta**2 - epsilon.transpose() *epsilon) * I + 2*(epsilon*epsilon.transpose()) -2*eta*epsilon_cross
        p = -1 * R * p
        T[0:3,0:3] = R.transpose()
        T[0:3,:3] = p
        T[:3,0:3] = np.zeros(shape=(1,3))
        T[3,3] = 1.0
        return T

    def get_transform(self, src_frame, tgt_frame):
        got_tf = false
        timeout = 0.025
        tf_temp
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        while(!got_tf and !rospy.is_shutdown()):
            try:
                tf_temp = tfBuffer.lookupTransform(src_frame, tgt_frame, rospy.Time(0), rospy.Duration(timeout))
                got_tf = True
            except:
                got_tf = False
                  rospy.logwarn_throttle(1, "Transform from " + src_frame + " to " + tgt_frame + " was not found! Is TF tree and clock being published?")
            if got_tf:
                T = self.convert_to_eigen(tf_temp)
        return T

class TriangulationNode:
    def set_camera_matrix(self, P_):
        P_ = P
        return True

    def set_node_name(self):
        node_name = rospy.get_name()
        return True

    def initialize_transforms(self):
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf.TransformListener()
        Tvi = transforms.get_transform()
        return True

def callback(img, odom):
    bridge = CvBridge()
    start = time.monotonic()
    img_save = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
    data = open(os.path.join(directory,triang_file))
    mycsv = csv.writer(data, 'a')
    if collect_image == 1:
        rospy.Rate(image_rate)
        filetime = rospy.rostime.Time.now.secs()
        filename = folder + str(filetime) + ".png"
        cv2.imwrite(filename, filename)
        Toi = transforms.get_odom_tf(odom)
        T_ic = transforms.get_transform(tfBuffer, camera_frame, imu_frame)
        E = np.zeros(shape =(4,4))
        E = transforms.get_inverse_tf(Toi * Tic)
        P_tilda = np.reshape(np.matmul(P_d * E), (-1,16))
        Tio = np.reshape(Tio, (-1,16))
        Pose = np.matrix([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
        row = [filename, P_tilda, Tio, Pose]
        mycsv.writerow(row)
        mycsv.close()
    stop = time.monotonic()
    elapsed = stop - start
    rospy.logdebug("[OBJ] TRIANGULATION TIME: " + str(elasped))
    return True

def get_ros_parameters():
    rospy.get_param(node_name + "/camera_frame",        camera_frame)
    rospy.get_param(node_name + "/image_rate",          image_rate)
    rospy.get_param(node_name + "/triang_file",         triang_file)
    rospy.get_param(node_name + "/folder",              folder)
    rospy.get_param(node_name + "/collect_image",       collect_image)
    return True
