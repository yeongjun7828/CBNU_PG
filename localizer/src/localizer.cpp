#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

#include <iostream>

static ros::Publisher pose_publisher;

static geometry_msgs::PoseWithCovariance _prev_pose;
static geometry_msgs::Quaternion _quat;
static double yaw;
// true if position history is long enough to compute orientation
static bool _orientation_ready = false;
// c-track (LiDAR map)
double utmoffsetX = 360951.74;
double utmoffsetY = 4065819.35;

static void utmcallback(const geometry_msgs::PoseStamped::ConstPtr& data)
{

  static tf::TransformBroadcaster pose_broadcaster;
  tf::Transform pose_transform;
  tf::Quaternion pose_q;

  geometry_msgs::PoseWithCovariance pose;

  pose.pose.position.x = data->pose.position.x;
  pose.pose.position.y = data->pose.position.y;
  pose.pose.position.z = 0;

  double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) +
                         pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));
  std::cout << "distance : " << distance << std::endl;

  if (distance > 0.3)
  {
    yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y, pose.pose.position.x - _prev_pose.pose.position.x);
    _quat = tf::createQuaternionMsgFromYaw(yaw);
    _prev_pose = pose;
    _orientation_ready = true;
  }

  if (_orientation_ready)
  {
    std::cout << "Yaw : " << yaw * 180 / M_PI << std::endl;
    pose.pose.orientation = _quat;
    pose_publisher.publish(pose);
  }

  static tf::TransformBroadcaster map_gps;

  map_gps.sendTransform(
  tf::StampedTransform(
  tf::Transform(tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w), tf::Vector3(pose.pose.position.x - utmoffsetX, pose.pose.position.y - utmoffsetY, 0.0)),
  ros::Time::now(),"map", "ego_vehicle"));
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "utm_yaw");
  ros::NodeHandle nh;
  
  ros::Rate rate(30);
  pose_publisher = nh.advertise<geometry_msgs::PoseWithCovariance>("/pose", 10);
  ros::Subscriber gnss_pose_subscriber = nh.subscribe("/utm", 10, utmcallback);
  ros::spin();
  rate.sleep();
  return 0;
}
