// Author: Keenan Burnett
// Copyright (C) 2020 aUToronto
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef TRIANGULATION_TRIANGULATION_NODE_H
#define TRIANGULATION_TRIANGULATION_NODE_H
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <chrono>  // NOLINT [build/c++11]
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include "utils/transform_utils.hpp"

//* Triangulation Node
class TriangulationNode {
 public:
    TriangulationNode(ros::NodeHandle nh) : nh_(nh) {}
    void set_camera_matrix(Eigen::Matrix4f P_) {P = P_;}
    void set_node_name() {node_name = ros::this_node::getName();}

    void callback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);

    /*!
       \brief Initializes the static transforms so that they only need to be queried once.
       \pre Make sure to run get_ros_parameters() before this.
    */
    void initialize_transforms();

    /*!
       \brief Retrieve rosparams that apply to this node.
       \pre Make sure to run set_node_name() before this function.
    */
    void get_ros_parameters();

 private:
    ros::NodeHandle nh_;
    zeus_tf::tfBufferPtr tfBuffer;
    zeus_tf::tfListenerPtr tfListener;
    std::string node_name = "triangulation";
    Eigen::Matrix4d Tcv = Eigen::Matrix4d::Identity();      /*!< Transform from velodyne to camera_frame */
    Eigen::Matrix4d Tvi = Eigen::Matrix4d::Identity();      /*!< Transform from imu_link to velodyne */
    Eigen::Matrix4f P = Eigen::Matrix4f::Identity();        /*!< Camera projection matrix */
    Eigen::Matrix4d Tio = Eigen::Matrix4d::Identity();     /*!< Transform from odom to imu_link */
    Eigen::Matrix4d Toi = Eigen::Matrix4d::Identity();     /*!< Transform from imu_link to odom */
    Eigen::Matrix4d Tci = Eigen::Matrix4d::Identity();     /*!< Transform from imu_link to camera_frame */
    Eigen::Matrix4d Tic = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d E = Eigen::Matrix4d::Identity();     /*!< Overall transformation matrix */
    Eigen::Matrix4d P_d = Eigen::Matrix4d::Identity();     /*!< Overall transformation matrix in float form */
    Eigen::Matrix4d P_tilda = Eigen::Matrix4d::Identity();   /*!< Camera matrix to be used in  triangulation */
    int cnt = 0;
    int collect_image = 1;
    std::string camera_frame = "mono_link";
    int image_rate = 3;
    std::string triang_file = "/home/autoronto/triangulation/data_trig_ped1.csv";
    std::string folder = "/home/autoronto/triangulation/triang_images_ped1/";


};

#endif  // TRIANGULATION_NODE_H
