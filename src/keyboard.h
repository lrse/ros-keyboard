#ifndef __ROS_KEYBOARD_H__
#define __ROS_KEYBOARD_H__

#include <keyboard/Key.h>

namespace keyboard {
  class Keyboard {
    public:
      Keyboard(void);
      ~Keyboard(void);

      int get_key(void);
  };    
}

#endif
