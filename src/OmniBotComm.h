#ifndef OMNIBOT_COMM
#define OMNIBOT_COMM
#include <stdint.h>

struct RobotPose {
  double t;

  double x;
  double y;
  double theta;

  double vx;
  double vy;
  double vth;
};


struct DrivePacket_Joystick {
  int8_t axis[6];
  uint16_t buttons;
};


struct DrivePacket {
  uint16_t packetIndex;
  uint8_t control;
  uint8_t dsDigitalIn;
  uint16_t teamID;
  uint8_t dsAlliance;
  uint8_t dsPosition;
/*
  DrivePacket_Joystick joystick1;
  DrivePacket_Joystick joystick2;
  DrivePacket_Joystick joystick3;
  DrivePacket_Joystick joystick4;
*/
};

#endif
