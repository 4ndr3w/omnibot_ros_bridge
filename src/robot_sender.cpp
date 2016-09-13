#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <zlib.h>

#include "OmniBotComm.h"

int sock = -1;
sockaddr_in addr;

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


void commandVelocity(const geometry_msgs::Twist::ConstPtr& msg)
{
  RobotMoveCommand cmd = {MODE_VELOCITY, byteswap(msg->linear.y), byteswap(msg->linear.x), 0};
  if ( sendto(sock, &cmd, sizeof(RobotMoveCommand), 0, (sockaddr*)&addr, sizeof(sockaddr_in)) < 0 )
    ROS_ERROR("commandVelocity couldn't send move command to robot controller");
}

void commandPosition(const geometry_msgs::Twist::ConstPtr& msg)
{
  RobotMoveCommand cmd = {MODE_POSITION, byteswap(msg->linear.y), byteswap(msg->linear.x), 0};
  if ( sendto(sock, &cmd, sizeof(RobotMoveCommand), 0, (sockaddr*)&addr, sizeof(sockaddr_in)) < 0 )
    ROS_ERROR("commandPosition couldn't send move command to robot controller");
}

/*
 * robot_sender node
 * Forwards ROS position/velocity commands to the robot controller
 */
int main(int argc, char** argv){
  ros::init(argc, argv, "robot_sender");

  ros::NodeHandle n;

  if ( (sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
    perror("socket() failed");
    ros::shutdown();
    exit(-1);
  }

  memset(&addr, 0, sizeof(sockaddr_in));

  n.subscribe("cmd_velocity", 100, commandVelocity);
  n.subscribe("cmd_position", 100, commandPosition);

  // Setup the robot's addr struct
  addr.sin_addr.s_addr = inet_addr("10.49.77.2");
  addr.sin_family = AF_INET;
  addr.sin_port = htons(1338);

  ros::spin();

  close(sock);
}
