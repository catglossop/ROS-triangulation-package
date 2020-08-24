// Author: Catherine Glossop
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <fstream>
#include <triangulation/triangulation_node.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>
#include "utils/geometry_utils.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "triangulation");
    ros::NodeHandle nh;
    // Get parameters
    std::string camera_frame, camera_topic, odom, output_append;
    std::string node_name = ros::this_node::getName();
    std::string triang_file, folder;
    int collect_image;
    nh.getParam(node_name + "/camera_frame",    camera_frame);
    nh.getParam(node_name + "/camera_topic",    camera_topic);
    nh.getParam(node_name + "/output_append",   output_append);
    nh.getParam(node_name + "/triang_file",     triang_file);
    nh.getParam(node_name + "/folder",          folder);
    nh.getParam(node_name + "/odom",            odom);
    nh.getParam(node_name + "/collect_image",    collect_image);
    // Subscribers
    message_filters::Subscriber<sensor_msgs::Image> sub_img(nh, camera_topic, 4);
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom(nh, odom, 40);
    // Initialize node object
    TriangulationNode myNode(nh);
    myNode.set_node_name();
    myNode.get_ros_parameters();
    myNode.initialize_transforms();
    std::ofstream myFile(triang_file);
    XmlRpc::XmlRpcValue P;
    myFile << " Filename; P; Tio; Pose\n";
    myFile.close();
    nh.getParam(node_name + "/P", P);
    Eigen::Matrix4f CAM;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double temp = P[4 * i + j];
            CAM(i, j) = static_cast<float>(temp);
        }
    }
    myNode.set_camera_matrix(CAM);
    // Synchronize subscribers
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> sync_policy;
    message_filters::Synchronizer<sync_policy> sync(sync_policy(100), sub_img, sub_odom);
    sync.registerCallback(boost::bind(&TriangulationNode::callback, myNode, _1, _2));

    ROS_INFO("[OBJ] Triangulation running!");
    ros::spin();
    return 0;
}
