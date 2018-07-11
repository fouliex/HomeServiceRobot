#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

double odom_x=0.0, odom_y=0.0;
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);

  // Add odom Subscription
  ros::Subscriber odom_sub;
  odom_sub = n.subscribe("odom", 1000, odom_callback);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);



  while (ros::ok())
  {
    visualization_msgs::Marker marker;

    // Set the frame ID and timpestand.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for the marker. This services to create a unique ID
    // Any  marker send with the same  namespace and id will  overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set our initial shape type to be a ARROW
    uint32_t shape = visualization_msgs::Marker::ARROW;
    marker.type = shape;

    //Set the marker action. Options are ADD, DELETE and DELETEALL
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 4.0;
    marker.pose.position.y = 2.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker 1x1x here means 1m on a  side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    // Initially show the marker at the pickup zone
    ROS_INFO("Robot is traveling to the pickup zone");
    while (marker_pub.getNumSubscribers() < 1){
      if (!ros::ok()){
	return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);


    ros::Duration(1.0).sleep();

    double marker_dist = 100.0;
    const double dist_threshold = 0.3;
    while( marker_dist > dist_threshold ){
      marker_dist = sqrt( pow(odom_x-marker.pose.position.x, 2) + pow(odom_y-marker.pose.position.y, 2) );
      ros::spinOnce();
      ros::Duration(1.0).sleep();
    }

    // Hide the marker once your robot reaches the pickup zone
    ROS_INFO("Robot picked up the virtual object");
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    // Wait 5 seconds to simulate a pickup
    ROS_INFO("Waiting for 5 seconds");
    ros::Duration(5.0).sleep();


   ROS_INFO("Robot is traveling to the drop off zone");


    // Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 4.0;
    marker.pose.position.y = -0.1;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker 1x1x here means 1m on a  side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    double drop_dist = 100.0;
    while( drop_dist > dist_threshold ){
      drop_dist = sqrt( pow(odom_x-marker.pose.position.x, 2) + pow(odom_y-marker.pose.position.y, 2) );
      ros::spinOnce();
      ros::Duration(1.0).sleep();
    }

    // Set the marker action again.
    ROS_INFO("Robot dropped the virtual object");
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
    
    ROS_INFO("Waiting for 10 seconds");
    ros::Duration(10.0).sleep();

   return 0;
  }
}
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  odom_x = odom_msg->pose.pose.position.x;
  odom_y = odom_msg->pose.pose.position.y;
}
