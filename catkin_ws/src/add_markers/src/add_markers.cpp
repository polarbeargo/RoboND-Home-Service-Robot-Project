#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_subscriber = n.subscribe("/odom", 1000, callback_odom);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  double start[2] = {3.0, 2.0};
  double end[2] = {-3.96, 6.14};

  
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = start[0];
  marker.pose.position.y = start[1];
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);
  ROS_INFO("Published marker at pick-up point");
  
  ros::Duration(5).sleep();
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  ROS_INFO("Removed marker from pick-up point");

  marker.pose.position.x = end[0];
  marker.pose.position.y = end[1];
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);
  ROS_INFO("Published marker at drop_off point");
  // Sleep for 5 sec
  ros::Duration(5.0).sleep();
  while(ros::ok()){
      ros::spinOnce();
      r.sleep();
  }
}