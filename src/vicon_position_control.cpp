#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;
ros::Publisher mavros_pose_pub; 
ros::Subscriber vicon_sub;

void vicon_cb(const geometry_msgs::PoseStamped &vicon)
{
  geometry_msgs::PoseStamped transformed_pose; 
  
  transformed_pose.header.stamp = ros::Time::now();
  transformed_pose.header.seq = vicon.header.seq;
  //transformed_pose.header.frame_id = "base_link_frd";

  //transformed_pose.header = vicon.header;

  transformed_pose.pose.position.x = 0.0;//vicon.pose.position.x; //rotates mocap NUE frame to ENU frame for MAVROS
  transformed_pose.pose.position.y = 0.0;//-vicon.pose.position.z;
  transformed_pose.pose.position.z = -0.1;//vicon.pose.position.y;

  transformed_pose.pose.orientation.x = 0;//vicon.pose.orientation.x; //Q:is it okay to just swap the quaternion components also?  
  transformed_pose.pose.orientation.y = 0;//-vicon.pose.orientation.z; //A:According to px4 documentation it is
  transformed_pose.pose.orientation.z = 0;//vicon.pose.orientation.y;
  transformed_pose.pose.orientation.w = 1;//vicon.pose.orientation.w;

  mavros_pose_pub.publish(transformed_pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vicon_node");
  ros::NodeHandle nh;
  mavros_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",2);
  vicon_sub = nh.subscribe("/vrpn_client_node/act_top/pose",2,vicon_cb);  

  geometry_msgs::PoseStamped setpoint;
  int count = 1;

  while(ros::ok())
  {
    ros::spinOnce();
  }
}
