#include <ros/ros.h>

#include "keyboard.h"

using namespace std;

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "keyboard");
  ros::NodeHandle n("~");

  ros::Publisher pub_down = n.advertise<keyboard::Key>("keydown", 10);
  ros::Publisher pub_up = n.advertise<keyboard::Key>("keyup", 10);

  // Load optional parameters for customizing window appearance
  int window_width, window_height, font_size;
  string window_caption, shown_text, text_font;

  n.param("window_width", window_width, 100);
  n.param("window_height", window_height, 100);
  n.param("window_caption", window_caption, string("ROS keyboard input"));
  n.param("shown_text", shown_text, string(""));
  n.param("text_font", text_font, string("/usr/share/fonts/truetype/freefont/FreeMono.ttf"));
  n.param("font_size", font_size, 14);

  try
  {
    keyboard::Keyboard kbd(window_width, window_height, window_caption, shown_text, text_font, font_size);
    ros::Rate r(50);

    keyboard::Key k;
    bool pressed, new_event;
    while (ros::ok() && kbd.get_key(new_event, pressed, k.code, k.modifiers))
    {
      if (new_event)
      {
        k.header.stamp = ros::Time::now();
        if (pressed)
          pub_down.publish(k);
        else
          pub_up.publish(k);
      }
      ros::spinOnce();
      r.sleep();
    }
  }
  catch (std::runtime_error& err)
  {
    ROS_ERROR("ROS keyboard initialization failed: %s", err.what());
    ros::shutdown();
  }

  ros::waitForShutdown();
}
