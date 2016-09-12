#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "OmniBotComm.h"

double byteswap(double v)
{
    union {
        uint64_t i;
        double  d;
    } conv;
    conv.d = v;
    conv.i = (conv.i & 0x00000000FFFFFFFF) << 32 | (conv.i & 0xFFFFFFFF00000000) >> 32;
    conv.i = (conv.i & 0x0000FFFF0000FFFF) << 16 | (conv.i & 0xFFFF0000FFFF0000) >> 16;
    conv.i = (conv.i & 0x00FF00FF00FF00FF) << 8  | (conv.i & 0xFF00FF00FF00FF00) >> 8;
    return conv.d;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time startTime = ros::Time::now();

  RobotPose robotPose;
  // Connect to robot controller
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in addr;
  memset(&addr, 0, sizeof(sockaddr_in));

  addr.sin_addr.s_addr = inet_addr("10.49.77.2");
  addr.sin_family = AF_INET;
  addr.sin_port = htons(1337);

  if ( connect(sock, (sockaddr*)&addr, sizeof(sockaddr_in)) == -1 )
    perror("Failed to connect to robot");
  memset(&robotPose, 0, sizeof(RobotPose));
  double firstT = -1;

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages

    if ( recv(sock, &robotPose, sizeof(RobotPose), 0) == -1 )
    {
      ros::shutdown();
      exit(1);
    }

    // Endian flip
    robotPose.t = byteswap(robotPose.t);

    robotPose.x = byteswap(robotPose.x);
    robotPose.y = byteswap(robotPose.y);
    robotPose.theta = byteswap(robotPose.theta);

    robotPose.vx = byteswap(robotPose.vx);
    robotPose.vy = byteswap(robotPose.vy);
    robotPose.vth = byteswap(robotPose.vth);

    if ( firstT == -1 )
	firstT = robotPose.t;

    ros::Time poseTime = startTime + ros::Duration(robotPose.t-firstT);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robotPose.theta);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = poseTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = robotPose.x;
    odom_trans.transform.translation.y = robotPose.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = poseTime;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = robotPose.x;
    odom.pose.pose.position.y = robotPose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = robotPose.vx;
    odom.twist.twist.linear.y = robotPose.vy;
    odom.twist.twist.angular.z = robotPose.vth;

    //publish the message
    odom_pub.publish(odom);
  }

  close(sock);
}
