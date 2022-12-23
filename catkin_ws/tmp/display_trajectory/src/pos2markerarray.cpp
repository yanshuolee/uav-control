#include "ros/ros.h"
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <stdio.h>

ros::Publisher pub_t265_path, pub_t265_drone, 
               pub_mavr_path, pub_mavr_drone;
nav_msgs::Path path_vins, path_mav;
bool init = true;
double init_x, init_y, init_z;

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    // Publish Path
    path_vins.header = pose_msg->header;
    path_vins.header.frame_id = "t265_link";

    geometry_msgs::PoseStamped pose;
    pose.header = pose_msg->header;
    pose.pose = pose_msg->pose.pose;
    /*
    pose.pose.position.x = pose_msg->pose.pose.position.y;
    pose.pose.position.y = -pose_msg->pose.pose.position.x;
    */
    path_vins.poses.push_back(pose);
    pub_t265_path.publish(path_vins);
    
    // Publish Markers
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header = pose_msg->header;
    car_mesh.header.frame_id = "t265_link";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    //car_mesh.type = 3;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://display_trajectory/model/drone.dae";

    car_mesh.pose = pose_msg->pose.pose;
    //car_mesh.pose.position.x = pose_msg->pose.pose.position.y;
    //car_mesh.pose.position.y = -pose_msg->pose.pose.position.x;
    
    Eigen::Matrix3d rot;
    rot << 0, 0, 1, 1, 0, 0, 0, 1, 0; // t265
    
    Eigen::Quaterniond Q;
    Q.w() = car_mesh.pose.orientation.w;
    Q.x() = car_mesh.pose.orientation.x;
    Q.y() = car_mesh.pose.orientation.y;
    Q.z() = car_mesh.pose.orientation.z;
    Q = Q * rot;
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();
    
    car_mesh.color.a = 0.7;
    car_mesh.color.r = 164.0/255.0;
    car_mesh.color.g = 222.0/255.0;
    car_mesh.color.b = 244.0/255.0;

    float major_scale = 0.001;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_t265_drone.publish(markerArray_msg);
}

void mavros_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    // Publish Path
    path_mav.header = pose_msg->header;
    path_mav.header.frame_id = "t265_link";
    
    geometry_msgs::PoseStamped pose;
    pose.header = pose_msg->header;
    pose.pose = pose_msg->pose;
    
    path_mav.poses.push_back(pose);
    pub_mavr_path.publish(path_mav);
    
    ROS_INFO("%6.3f | %6.3f | %6.3f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    
    // Publish Markers
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header = pose_msg->header;
    car_mesh.header.frame_id = "t265_link";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    //car_mesh.type = 2;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://display_trajectory/model/drone.dae";

    car_mesh.pose = pose_msg->pose;
    
    Eigen::Matrix3d rot;
    //rot << -1, 0, 0, 0, -1, 0, 0, 0, 1; // vins fusion marvelmind
    //rot << 0, 0, 1, 1, 0, 0, 0, 1, 0; // vins_450
    rot << 0, 0, 1, 1, 0, 0, 0, 1, 0; // t265 fusion marvelmind
    //rot << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    
    Eigen::Quaterniond Q;
    Q.w() = car_mesh.pose.orientation.w;
    Q.x() = car_mesh.pose.orientation.x;
    Q.y() = car_mesh.pose.orientation.y;
    Q.z() = car_mesh.pose.orientation.z;
    Q = Q * rot;
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();
    
    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 1.0;
    car_mesh.color.b = 0.0;

    float major_scale = 0.001;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_mavr_drone.publish(markerArray_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_display");
    ros::NodeHandle n;

    ros::Subscriber sub_t265 = n.subscribe("/t265/odom/sample", 100, vio_callback);
    ros::Subscriber sub_mav  = n.subscribe("/mavros/local_position/pose", 100, mavros_callback);
    pub_t265_path  = n.advertise<nav_msgs::Path>("display/t265/path", 1000);
    pub_t265_drone = n.advertise<visualization_msgs::MarkerArray>("display/t265/drone_model", 1000);
    pub_mavr_path  = n.advertise<nav_msgs::Path>("display/mavros/path", 1000);
    pub_mavr_drone = n.advertise<visualization_msgs::MarkerArray>("display/mavros/drone_model", 1000);
    ros::spin();
    return 0;
}

