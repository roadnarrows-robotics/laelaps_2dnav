#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "laelaps_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(20);

  tf::TransformBroadcaster broadcaster;
  
  while(n.ok()){
    //This should be published by gazebo, but robot state publisher is not 
    //publishing this transform. TODO: fix this
    broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.0, 0.0, 0.09)),
          ros::Time::now(), "base_footprint", "base_link"));
    //This would typically be published by AMCL but for simulation we are
    //simplifying by assuming perfect odometry
    broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.0, 0.0, 0.0)),
          ros::Time::now(), "/map", "/odom"));
    //Since our virtual cloud is in world coords we can assume it is 
    //in the map frame
    broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.0, 0.0, 0.0)),
          ros::Time::now(), "/map", "/cloud"));
    
    r.sleep();
  }
}
