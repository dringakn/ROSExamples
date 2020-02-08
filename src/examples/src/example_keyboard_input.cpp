/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <termios.h> /* Terminal device*/

struct termios old_tio, new_tio; /* Store old and new terminal settings*/
unsigned char getKeyboard(unsigned char buff[3]) {
  unsigned char chrCounts = 0;             /* Buffer to input character*/
  memset(buff, 0, 3);                      /* Clear buffer*/
  chrCounts = read(STDIN_FILENO, buff, 3); /* Read the character*/
  // tcflush(STDIN_FILENO, TCIFLUSH);            /* Remove buffered characters*/
  return chrCounts;
}

geometry_msgs::Twist mapTwist(float tarLinVel, float tarAngVel, float acc) {
  static geometry_msgs::Twist prevCmd;
  static ros::Time prevTime = ros::Time::now();
  ros::Time currTime = ros::Time::now();
  float delta = acc * (currTime - prevTime).toSec();
  prevTime = currTime;
  float deltaLinVel = fabs(tarLinVel - prevCmd.linear.x);
  float deltaAngVel = fabs(tarAngVel - prevCmd.angular.z);
  if (deltaLinVel <= delta)
    prevCmd.linear.x = tarLinVel;
  else
    prevCmd.linear.x += (tarLinVel > prevCmd.linear.x) ? delta : -delta;
  if (deltaAngVel <= delta)
    prevCmd.angular.z = tarAngVel;
  else
    prevCmd.angular.z += (tarAngVel > prevCmd.angular.z) ? delta : -delta;
  return prevCmd;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "keyboard_input");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  geometry_msgs::Twist currCmd, prvCmd;
  ros::Rate rate(20);
  unsigned char chars,
      keyboardBuff[3]; /* Maximum three characters are returned for special
                          keys*/
  float linVel = 0, angVel = 0, linVel_Scale = 1, angVel_Scale = 1, acc = 0.01;
  float tarLinVel = linVel, tarAngVel = angVel;
  ros::Time currTime, prevTime = ros::Time::now();

  tcgetattr(STDIN_FILENO, &old_tio); /* get the terminal settings for stdin */
  new_tio =
      old_tio; /* we want to keep the old setting to restore them a the end */
  new_tio.c_lflag &= ~ICANON; /* disable canonical mode (buffered i/o) */
  new_tio.c_lflag &= ~ECHO;   /* disable local echo */
  new_tio.c_cc[VMIN] = 0;     /* One character at a time*/
  new_tio.c_cc[VTIME] = 0;    /* Inter charcter timer off*/
  tcsetattr(STDIN_FILENO, TCSANOW,
            &new_tio); /* set the new settings immediately */

  ROS_WARN("Use arrow keys to publish Twist message.");
  ROS_WARN("Use q/z to increase/decrease linear velocity.");
  ROS_WARN("Use w/x to increase/decrease angular velocity.");
  ROS_WARN("Use e/c to increase/decrease linear and angular acceleration.");
  if (!nh.hasParam("linVel_Scale")) {
    ROS_INFO("Parameter (linVelScale) doesn't exist, creating with default "
             "value: 1.0");
    nh.setParam("linVel_Scale", linVel_Scale);
  }
  if (!nh.hasParam("angVel_Scale")) {
    ROS_INFO("Parameter (angVelScale) doesn't exist, creating with default "
             "value: 1.0");
    nh.setParam("angVel_Scale", angVel_Scale);
  }
  if (!nh.hasParam("acc")) {
    ROS_INFO("Parameter (acc) doesn't exist, creating with default value: 1.0");
    nh.setParam("acc", acc);
  }

  while (ros::ok()) {
    chars = getKeyboard(keyboardBuff);
    if (chars == 1) /* Standard character */
    {
      switch (keyboardBuff[0]) {
      case 'q':
      case 'Q':
        if (linVel_Scale < 1e6)
          linVel_Scale += 0.1;
        nh.setParam("linVel_Scale", linVel_Scale);
        break;
      case 'z':
      case 'Z':
        if (linVel_Scale >= 0.1)
          linVel_Scale -= 0.1;
        nh.setParam("linVel_Scale", linVel_Scale);
        break;
      case 'w':
      case 'W':
        if (angVel_Scale < 1e6)
          angVel_Scale += 0.1;
        nh.setParam("angVel_Scale", angVel_Scale);
        break;
      case 'x':
      case 'X':
        if (angVel_Scale >= 0.1)
          angVel_Scale -= 0.1;
        nh.setParam("angVel_Scale", angVel_Scale);
        break;
      case 'e':
      case 'E':
        if (acc < 1e6)
          acc += 0.01;
        nh.setParam("acc", acc);
        break;
      case 'c':
      case 'C':
        if (acc >= 0.01)
          acc -= 0.01;
        nh.setParam("acc", acc);
        break;
      default:
        break;
      }
      ROS_INFO("linVel_Scale:%05.2f angVel_Scale:%05.2f acc:%05.2f",
               linVel_Scale, angVel_Scale, acc);
    } else if (chars == 3) /* Special character*/
    {
      if (keyboardBuff[0] == 27 && keyboardBuff[1] == 91) {
        switch (keyboardBuff[2]) {
        case 65: /* UP */
          linVel = 1;
          angVel = 0;
          break;
        case 66: /* DOWN */
          linVel = -1;
          angVel = 0;
          break;
        case 67: /* RIGHT */
          linVel = 0;
          angVel = 1;
          break;
        case 68: /* LEFT */
          linVel = 0;
          angVel = -1;
          break;
        default:
          break;
        }
      }
    } else /* No character*/
    {
      linVel = 0;
      angVel = 0;
    }
    nh.getParamCached("linVel_Scale", linVel_Scale);
    nh.getParamCached("angVel_Scale", angVel_Scale);
    nh.getParamCached("acc", acc);
    tarLinVel = linVel * linVel_Scale;
    tarAngVel = angVel * angVel_Scale;
    pub.publish(mapTwist(tarLinVel, tarAngVel, acc));
    // ROS_INFO("keyboard: %d %d %d %d", chars, keyboardBuff[0],
    // keyboardBuff[1], keyboardBuff[2]);
    rate.sleep();
    ros::spinOnce();
  }
  tcsetattr(STDIN_FILENO, TCSANOW, &old_tio); /* Restore the former settings */
  return 0;
}
