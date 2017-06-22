#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "tf/tf.h"
#include "tf/transform_listener.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void addWall(double x_min, double x_max, double y_min, double y_max,
             double z_min, double z_max, double inc, PointCloud::Ptr &pIn){
    if(x_max-x_min <= 0){
        for(int i=1; i<(int)((z_max - z_min)/inc); i++){
            for(int j=1; j<(int)((y_max - y_min)/inc); j++){
                pIn->points.push_back(pcl::PointXYZ(x_min, (j*inc+y_min), (i*inc+z_min)));
            }
        }
    }else{
        for(int i=1; i<(int)((z_max - z_min)/inc); i++){
            for(int j=1; j<(int)((x_max - x_min)/inc); j++){
                pIn->points.push_back(pcl::PointXYZ((j*inc+x_min), y_min, (i*inc+z_min)));
            }
        }
    }
}

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


    double x = 10.0;
    double min_y = -5.0;
    double max_y = 5.0;
    double min_z = 0.1;
    double max_z = 0.9;
    double y_inc = 0.01;
    double z_inc = 0.01;
    int count = 0;
    ROS_INFO("Populate Pointcloud");
    addWall(10.0, 10.0, -5.0, 5.0, 0.1, 0.9, 0.2, pIn);
    addWall(-10.0, -10.0, -5.0, 5.0, 0.1, 0.9, 0.2, pIn);
    addWall(-10.0, 10.0, -5.0, -5.0, 0.1, 0.9, 0.2, pIn);
    addWall(-10.0, 10.0, 5.0, 5.0, 0.1, 0.9, 0.2, pIn);
    pIn->width = pIn->points.size();

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
