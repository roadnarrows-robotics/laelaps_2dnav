#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "tf/tf.h"
#include "tf/transform_listener.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv){
    ros::init(argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("/cloud_in", 1);
    tf::TransformListener m_tfListener;
    ROS_INFO("Init Pointclouds");
    PointCloud::Ptr pIn(new PointCloud);
    PointCloud::Ptr pOut(new PointCloud);
    pIn->header.frame_id = "map";
    pIn->height = 1;


    double x = 1.0;
    double min_y = -5.0;
    double max_y = 5.0;
    double min_z = 0.1;
    double max_z = 0.9;
    double y_inc = 0.01;
    double z_inc = 0.01;
    int count = 0;
    ROS_INFO("Populate Pointcloud");
    for(int i=1; i<(int)((max_z - min_z)/z_inc); i++){
        for(int j=1; j<(int)((max_y - min_y)/y_inc); j++){
            pIn->points.push_back(pcl::PointXYZ(x, (j*y_inc+min_y), (i*z_inc+min_z)));
            count++;
        }
    }
    pIn->width = count;

    ros::Rate loop_rate(20);
    ROS_INFO("Start loop");
    while(nh.ok()){
        try{
          m_tfListener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(3.0));
          pcl_ros::transformPointCloud("base_footprint", *pIn, *pOut, m_tfListener);
        }
        catch(tf::TransformException ex){
          ROS_ERROR("Received exception trying to transform point from map to base_footprint: %s", ex.what());
        }
        pOut->header.stamp = ros::Time::now().toNSec()/1000.0;
        pub.publish(pOut);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
