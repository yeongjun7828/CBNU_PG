#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <std_msgs/Int16.h>

class localizer {
public:
    localizer() {
      nh_.param<std::string>("imu_topic", imu_topic, "/imu/data");
      nh_.param<std::string>("utm_topic", utm_topic, "/utm");
      nh_.param<std::string>("gps_topic", gps_topic, "/ublox_gps/fix");

      imu_sub_ = nh_.subscribe(imu_topic, 10, &localizer::imucallback, this);
      filtered_imu_sub_ = nh_.subscribe("/livox/imu", 10, &localizer::filteredimucallback, this);
      utm_sub_ = nh_.subscribe(utm_topic, 10, &localizer::utmcallback, this);
      gps_sub_ = nh_.subscribe(gps_topic, 10, &localizer::gpscallback, this);

      pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovariance>("/imu_pose", 8);
      // fpose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovariance>("/fpose", 8);
      // c-track
      utmoffsetX = 360777.923575;
      utmoffsetY = 4065980.612646;
    }


    void filteredimucallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
      filtered_imu_data_ = *imu_msg;

      tf::Quaternion filtered_quat;
      tf::quaternionMsgToTF(imu_msg->orientation, filtered_quat);
      
      tf::Matrix3x3(filtered_quat).getEulerYPR(filtered_yaw, filtered_pitch, filtered_roll);
    }


    void imucallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
      imu_data_ = *imu_msg;

      tf::Quaternion quat;
      tf::quaternionMsgToTF(imu_msg->orientation, quat);
      
      tf::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);
    }

    void utmcallback(const geometry_msgs::PoseStamped::ConstPtr& utm_msg) {
      utm_x_ = utm_msg->pose.position.x;
      utm_y_ = utm_msg->pose.position.y;
    }

    void gpscallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
      gps_cov_ = gps_msg->position_covariance[0];
      gps_lat_ = gps_msg->latitude;
      gps_lon_ = gps_msg->longitude;

    }

    void pub_pose() {      
      geometry_msgs::PoseWithCovariance pose_msg;
      pose_msg.pose.position.x = utm_x_;
      pose_msg.pose.position.y = utm_y_;
      pose_msg.pose.position.z = 0.0;
      pose_msg.pose.orientation.x = imu_data_.orientation.x;
      pose_msg.pose.orientation.y = imu_data_.orientation.y;
      pose_msg.pose.orientation.z = imu_data_.orientation.z;
      pose_msg.pose.orientation.w = imu_data_.orientation.w;
      pose_msg.covariance[0] = gps_cov_;
      pose_pub_.publish(pose_msg);

      // geometry_msgs::PoseWithCovariance fpose_msg;
      // fpose_msg.pose.position.x = utm_x_;
      // fpose_msg.pose.position.y = utm_y_;
      // fpose_msg.pose.position.z = 0.0;
      // fpose_msg.pose.orientation.x = filtered_imu_data_.orientation.x;
      // fpose_msg.pose.orientation.y = filtered_imu_data_.orientation.y;
      // fpose_msg.pose.orientation.z = filtered_imu_data_.orientation.z;
      // fpose_msg.pose.orientation.w = filtered_imu_data_.orientation.w;
      // fpose_msg.covariance[0] = gps_cov_;
      // fpose_pub_.publish(fpose_msg);


      static tf::TransformBroadcaster map_imu;

      map_imu.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(imu_data_.orientation.x, imu_data_.orientation.y, imu_data_.orientation.z, imu_data_.orientation.w), tf::Vector3(pose_msg.pose.position.x - utmoffsetX, pose_msg.pose.position.y - utmoffsetY, 0.0)),
        // tf::Transform(tf::Quaternion(imu_data_.orientation.x, imu_data_.orientation.y, imu_data_.orientation.z, imu_data_.orientation.w), tf::Vector3(5.0, 5.0, 0.0)),
        // tf::Transform(tf::Quaternion(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w), tf::Vector3(pose_msg.pose.position.x - utmoffsetX, pose_msg.pose.position.y - utmoffsetY, 0.0)),
        ros::Time::now(),"map", "imu_ego_vehicle"));


      // markerId = 0;

      // visualization_msgs::Marker marker;
      // marker.header.frame_id = "map";
      // marker.header.stamp = ros::Time::now();
      // marker.ns = "pose";
      // marker.id = markerId++;
      // marker.type = visualization_msgs::Marker::ARROW;
      // marker.action = visualization_msgs::Marker::ADD;
      // marker.pose.position.x = utm_x_ - utmoffsetX;
      // marker.pose.position.y = utm_y_ - utmoffsetY;
      // marker.pose.position.z = 0;
      // marker.pose.orientation.x = imu_data_.orientation.x;
      // marker.pose.orientation.y = imu_data_.orientation.y;
      // marker.pose.orientation.z = imu_data_.orientation.z;
      // marker.pose.orientation.w = imu_data_.orientation.w;
      // marker.scale.x = 0.2;
      // marker.scale.y = 0.1;
      // marker.scale.z = 0.1;

      // if(markerId > 200) markerId = 0;
      
      // marker.color.r = 0.0f;
      // marker.color.g = 1.0f;
      // marker.color.b = 0.0f;
      // marker.color.a = 1.0;
      // marker_pub_.publish(marker);

      // Debugging      
      ROS_INFO("x: %f,   longitude: %f", utm_x_, gps_lon_);
      ROS_INFO("y: %f,   latitude: %f", utm_y_, gps_lat_);
      ROS_INFO("%f, %f, %f", roll, pitch, yaw);
      // ROS_INFO("Heading Angle: %f", yaw);
      if (gps_cov_ < 0.001){
        ROS_INFO("RTCM received in GPS");
      }
      else{
        ROS_WARN("RTCM not received in GPS");
        ROS_WARN("Covariance: %f", gps_cov_);
      }
      // cnt += 1;
      // if (cnt == 5) {
      //   std::system("clear");
      //   cnt = 0;
      // }
    }
private:
  std::string imu_topic;
  std::string utm_topic;
  std::string gps_topic;
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  ros::Subscriber filtered_imu_sub_;
  ros::Subscriber utm_sub_;
  ros::Subscriber gps_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher fpose_pub_;
  ros::Publisher marker_pub_;
  sensor_msgs::Imu imu_data_;
  sensor_msgs::Imu filtered_imu_data_;
  double utm_x_;
  double utm_y_;
  double gps_cov_;
  double gps_lat_;
  double gps_lon_;
  double roll, pitch, yaw;
  double filtered_roll, filtered_pitch, filtered_yaw;
  double utmoffsetX;
  double utmoffsetY;
  // int cnt = 0;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_localizer");

  localizer localizer;

  ros::Rate rate(10);

  while (ros::ok()) {
    localizer.pub_pose();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}