//
// Created by YJ on 22. 10. 3.
//

#ifndef SRC_PARKING_PATH_CTRACK_H
#define SRC_PARKING_PATH_CTRACK_H


#include <ros/ros.h>

#include <tf/tf.h>

#include <std_msgs/String.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/PointCloud.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose2D.h>

#include <iostream>
#include <vector>
#include <stack>
#include <istream>
#include <fstream>

struct path_data{
    nav_msgs::Path path;
    int cost;
};

class Simple_Path{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Timer timer;

    //Publisher
    ros::Publisher outpath_pub;
    ros::Publisher stop_area_pub;

    //Subscriber
    ros::Subscriber pose_sub;

    //Call Back
    void Timer_CB(const ros::TimerEvent& event);
    void parsing_data1(std::ifstream &fin);


    void pose_CB(const geometry_msgs::PoseWithCovariance::ConstPtr &msg);
    void path_CB();

    //launch param
    int parking_area_cnt;
    std::string path_dir;

    //package variable
    geometry_msgs::PoseWithCovariance pose;

    //flag
    bool path_flag;

    //Variable
    double offset_x, offset_y;
    double yaw;
    std::vector<path_data> path_with_cost;
    std::vector<nav_msgs::Path> path1;


public:
    Simple_Path();

};


#endif //SRC_PARKING_PATH_CTRACK_H
