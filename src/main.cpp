#include <ros/ros.h>
#include <iostream>
#include "keyboard.h"
using namespace std;

int main(int argc, char** argv)
{
  keyboard::Keyboard kbd;
  
  ros::init(argc, argv, "keyboard");
  ros::NodeHandle n("~");

  ros::Publisher pub = n.advertise<keyboard::Key>("keypress", 5);

  ros::Rate r(50);
  while (ros::ok()) {
    int c = kbd.get_key();
    if (c != -1) {
      keyboard::Key k;
      k.key = c;
      pub.publish(k);
    }
    ros::spinOnce();
    r.sleep();
  }
  
  ros::waitForShutdown();
}
