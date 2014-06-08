#include <ros/ros.h>
#include <iostream>
#include "keyboard.h"
using namespace std;

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "keyboard");
  ros::NodeHandle n("~");

  ros::Publisher pub_down = n.advertise<keyboard::Key>("keydown", 10);
  ros::Publisher pub_up = n.advertise<keyboard::Key>("keyup", 10);

  keyboard::Keyboard kbd;
  ros::Rate r(50);
  
  keyboard::Key k;
  bool pressed, new_event;
  while (ros::ok() && kbd.get_key(new_event, pressed, k.code, k.modifiers)) {
    if (new_event) {
      k.header.stamp = ros::Time::now();
      if (pressed) pub_down.publish(k);
      else pub_up.publish(k);
    }
    ros::spinOnce();
    r.sleep();
  }
  
  ros::waitForShutdown();
}
