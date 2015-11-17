#ifndef __ROS_KEYBOARD_H__
#define __ROS_KEYBOARD_H__

#include <keyboard/Key.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

namespace keyboard {
  class Keyboard {
    public:
      Keyboard(int window_width, int window_height, std::string window_caption,
               std::string shown_text, std::string text_font, int font_size);
      ~Keyboard(void);

      void render(void);
      bool get_key(bool& new_event, bool& pressed, uint16_t& code, uint16_t& modifiers);

    private:
      bool          use_ttf;
      SDL_Window*   window;
      SDL_Renderer* renderer;
      SDL_Surface*  surface;
      SDL_Texture*  texture;
  };
}

#endif
