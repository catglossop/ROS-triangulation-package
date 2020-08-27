// Author: Keenan Burnett
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include "triangulation/triangulation_node.h"
#include <opencv2/core.hpp>
#include <opencv/cv.hpp>


void TriangulationNode::callback(const sensor_msgs::ImageConstPtr& img,
    const nav_msgs::OdometryConstPtr& odom) {
    auto start = std::chrono::high_resolution_clock::now();
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
    cv::Mat img_save = cv_ptr->image;
    std::ofstream mycsv;
    mycsv.open(triang_file, std::ios::app);
    typedef Eigen::Matrix<float, 1, Eigen::Dynamic> rowVectorf;
    typedef Eigen::Matrix<double, 1, Eigen::Dynamic> rowVectord;
    if (collect_image == 1){
      ros::Rate r(image_rate);
      double filetime = ros::Time::now().toSec();
      std::string filename = folder + std::to_string(filetime) + ".png";
      cv::imwrite(filename, img_save);
      zeus_tf::get_odom_tf(*odom, Toi);
      zeus_tf::get_transform(tfBuffer, camera_frame, "imu_link", Tic);
      E = zeus_tf::get_inverse_tf(Toi * Tic);
      for (int i = 0; i < 4; i++) {
          for (int j = 0; j < 4; j++) {
            float temp;
            temp = P(i,j);
            P_d(i, j) = static_cast<double>(temp);
          }
      }
      P_tilda = P_d * E; // print to csv file
      P_tilda.transposeInPlace();
      Tio.transposeInPlace();
      Eigen::VectorXd P_vector(Eigen::Map<Eigen::VectorXd>(P_tilda.data(), P_tilda.cols()*P_tilda.rows()));
      Eigen::VectorXd T_vector(Eigen::Map<Eigen::VectorXd>(Tio.data(), Tio.cols()*Tio.rows()));
      rowVectord P_row = P_vector.transpose();
      rowVectord T_row = T_vector.transpose();
      Eigen::VectorXf Pose = Eigen::VectorXf::Constant(3,1,1);
      Pose << odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z;
      rowVectorf Pose_row = Pose.transpose();
      mycsv << std::setprecision(16) << filename << ";" << P_row << ";" << T_row << ";" << Pose_row << ";" << "\n";
      mycsv.close();
      r.sleep();
    }
    auto stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = stop - start;
    ROS_DEBUG_STREAM("[OBJ] TRIANGULATION TIME: " << elapsed.count());
}

void TriangulationNode::initialize_transforms() {
    tfBuffer = zeus_tf::tfBufferPtr(new zeus_tf::tfBuffer());
    tfListener = zeus_tf::tfListenerPtr(new zeus_tf::tfListener(*tfBuffer));
    zeus_tf::get_transform(tfBuffer, "imu_link", "velodyne", Tvi);
    zeus_tf::get_transform(tfBuffer, "velodyne", camera_frame, Tcv);
}

void TriangulationNode::get_ros_parameters() {
    nh_.getParam(node_name + "/camera_frame",            camera_frame);
    nh_.getParam(node_name + "/image_rate",              image_rate);
    nh_.getParam(node_name + "/triang_file",             triang_file);
    nh_.getParam(node_name + "/folder",                  folder);
    nh_.getParam(node_name + "/odom",                    odom);
    nh_.getParam(node_name + "/collect_image",           collect_image);
}
