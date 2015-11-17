#include "keyboard.h"

keyboard::Keyboard::Keyboard(int window_width, int window_height, std::string window_caption,
                             std::string shown_text, std::string text_font, int font_size)
                  : use_ttf(false), window(NULL), renderer(NULL), surface(NULL), texture(NULL)
{
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
    throw std::runtime_error("Could not init SDL");
  window = SDL_CreateWindow(window_caption.c_str(), SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                            window_width, window_height, SDL_WINDOW_SHOWN);
  renderer = SDL_CreateRenderer(window, -1, 0);

  if (! shown_text.empty())
  {
    // We have some text to show in the window, so we must use SDL_ttf
    use_ttf = true;

    // Initialize SDL_ttf library
    if (TTF_Init() != 0)
      throw std::runtime_error("Could not init SDL-TTF");

    // Load a font; WARN: at least on Ubuntu it requires the full path of the .ttf file
    TTF_Font *font = TTF_OpenFont(text_font.c_str(), font_size);
    if (font == NULL)
      throw std::runtime_error("Could not open TTF font: " + text_font);

    // Write text to surface; we use SDL2's Blended_Wrapped so line breaks are not ignored
    SDL_Color text_color = { 255, 255, 255 };
    surface = TTF_RenderText_Blended_Wrapped(font, shown_text.c_str(), text_color, window_width);
    if (surface == NULL)
      throw std::runtime_error("Could not create blended wrapped text surface");
    texture = SDL_CreateTextureFromSurface(renderer, surface);
    if (texture == NULL)
      throw std::runtime_error("Could not create texture from text surface");
  }
}

keyboard::Keyboard::~Keyboard(void)
{
  if (use_ttf)
  {
    SDL_DestroyTexture(texture);
    SDL_FreeSurface(surface);
    TTF_Quit();
  }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

void keyboard::Keyboard::render(void)
{
  if (use_ttf)
  {
    int texW = 0;
    int texH = 0;
    SDL_QueryTexture(texture, NULL, NULL, &texW, &texH);
    SDL_Rect dstrect = { 0, 0, texW, texH };
    SDL_RenderCopy(renderer, texture, NULL, &dstrect);
  }
  SDL_RenderPresent(renderer);
}

bool keyboard::Keyboard::get_key(bool& new_event, bool& pressed, uint16_t& code, uint16_t& modifiers)
{
  render();

  new_event = false;

  SDL_Event event;
  SDL_WaitEvent(&event);

  switch(event.type)
  {
    case SDL_KEYUP:
      pressed = false;
      code = event.key.keysym.sym;
      modifiers = event.key.keysym.mod;
      new_event = true;
    break;
    case SDL_KEYDOWN:
      pressed = true;
      code = event.key.keysym.sym;
      modifiers = event.key.keysym.mod;
      new_event = true;
    break;
    case SDL_QUIT:
      return false;
    break;
  }

  return true;
}

