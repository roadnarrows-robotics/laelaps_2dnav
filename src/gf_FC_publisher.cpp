#include "ros/ros.h"
#include "geofrenzy/GfDistFeatureCollection.h"
#include "geofrenzy/GfDistFeature.h"
#include "geofrenzy/Polygon64.h"
#include <geometry_msgs/PointStamped.h>
#include "tf/tf.h"
#include "tf/transform_listener.h"

/*
 * Publish gfFeatureCollection for testing
 */
/*
 * Setup test static Polygon64 for testing robot-centric map
 */

ros::Publisher fc_pub;

void publishFeatureCollection(){

    tf::TransformListener tf_listener;
    geometry_msgs::PointStamped new_point;
    geometry_msgs::PointStamped trans_point;
    geofrenzy::Polygon64 polygon;
    new_point.header.frame_id="map";
    new_point.header.stamp = ros::Time();
    new_point.point.x = -5.0;
    new_point.point.y = -5.0;
    new_point.point.z = 0.0;
    try{
      tf_listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(3.0));
      tf_listener.transformPoint("base_footprint", new_point, trans_point);
    }
    catch(tf::TransformException ex){
      ROS_ERROR("Received exception trying to transform point from map to base_footprint: %s", ex.what());
    }
    polygon.points.push_back(trans_point.point);

    new_point.point.x = 0.0;
    new_point.point.y = 5.0;
    new_point.point.z = 0.0;
    try{
      tf_listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(3.0));
      tf_listener.transformPoint("base_footprint", new_point, trans_point);
    }
    catch(tf::TransformException ex){
      ROS_ERROR("Received exception trying to transform point from map to base_footprint: %s", ex.what());
    }
    polygon.points.push_back(trans_point.point);

    new_point.point.x = 5.0;
    new_point.point.y = -5.0;
    new_point.point.z = 0.0;
    try{
      tf_listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(3.0));
      tf_listener.transformPoint("base_footprint", new_point, trans_point);
    }
    catch(tf::TransformException ex){
      ROS_ERROR("Received exception trying to transform point from map to base_footprint: %s", ex.what());
    }
    polygon.points.push_back(trans_point.point);

    geofrenzy::GfDistFeature distFeature;
    distFeature.geometry.push_back(polygon);

    geofrenzy::GfDistFeatureCollection fc;
    fc.header.stamp = ros::Time::now();
    fc.features.push_back(distFeature);

    fc_pub.publish(fc);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "fc_publisher");
    ros::NodeHandle n;
    fc_pub = n.advertise<geofrenzy::GfDistFeatureCollection>("/geofrenzy/168/featureCollection/distance", 10);
    ros::Rate loop_rate(10);
    while(ros::ok()){
        publishFeatureCollection();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
