#ifndef __ROS_KEYBOARD_H__
#define __ROS_KEYBOARD_H__

#include <keyboard/Key.h>
#include <SDL.h>

namespace keyboard {
  class Keyboard {
    public:
      Keyboard(void);
      ~Keyboard(void);

      bool get_key(bool& new_event, bool& pressed, uint16_t& code, uint16_t& modifiers);

    private:
      SDL_Surface* window;
  };    
}

#endif
