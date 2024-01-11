//
// Created by YJ on 22. 10. 3.
//

#include "simple_path_ctrack/simple_path_ctrack.h"

using namespace std;


Simple_Path::Simple_Path()
        :nh(""), pnh("~")
{
    pnh.getParam("offset_x", offset_x);
    pnh.getParam("offset_y", offset_y);
    pnh.getParam("path_dir", path_dir);

    timer = nh.createTimer(ros::Duration(0.1),&Simple_Path::Timer_CB,this);
    //Publisher
    outpath_pub = nh.advertise<nav_msgs::Path>("global_path",1);

    //Subscriber

    pose_sub = nh.subscribe("current_pose_topic_name",1,&Simple_Path::pose_CB,this);

    path_flag = false;
    if(path_flag == false)
    {
        path_CB();
        path_flag = true;
    }



}

void Simple_Path::path_CB() {
    ROS_INFO("path dir: %s", path_dir.c_str());
    ifstream tmp;

    tmp.open(path_dir + "/" + "path1.txt");
    parsing_data1(tmp);
    tmp.close();


}


void Simple_Path::pose_CB(const geometry_msgs::PoseWithCovariance::ConstPtr &msg) {
    pose = *msg;
    pose.pose.position.x -= offset_x;
    pose.pose.position.y -= offset_y;

    tf::Matrix3x3 matrix(tf::Quaternion(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w));
    double r,p;
    matrix.getRPY(r,p,yaw);
}

void Simple_Path::parsing_data1(std::ifstream &fin) {
    string line;

    int start, end;

    path_data temp_data;
    nav_msgs::Path outpath;
    geometry_msgs::Pose2D point;

    if (fin.is_open()) {
        outpath.header.stamp = ros::Time::now();
        outpath.header.frame_id = "map";

        std::vector<geometry_msgs::PoseStamped> waypoints;
        geometry_msgs::PoseStamped temp_pose;
        while (getline(fin, line)) {


            start = line.find("\t", 0);
            end = line.size() - 1;

            std::string utm_x = line.substr(0, start);
            std::string utm_y = line.substr(start + 1, end);

            temp_pose.pose.position.x = std::stod(utm_x.c_str()) - offset_x;
            temp_pose.pose.position.y = std::stod(utm_y.c_str()) - offset_y;
            
            waypoints.push_back(temp_pose);
        }
        outpath.poses = waypoints;
        path1.push_back(outpath);

    } else {
        std::cout << "Unable to open file" << std::endl;
    }
}

void Simple_Path::Timer_CB(const ros::TimerEvent &event) {


    if (path_flag == true){
        for (int i=0;i< path1.size();i++){
            outpath_pub.publish(path1.at(i));
        }
    }
}
