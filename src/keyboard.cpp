#include <ncurses.h>
#include "keyboard.h"

keyboard::Keyboard::Keyboard(void)
{
  initscr();
  clear();
  noecho();
  cbreak();
  timeout(0);
  keypad(stdscr, TRUE);
}

keyboard::Keyboard::~Keyboard(void)
{
  endwin();
}

int keyboard::Keyboard::get_key(void)
{
  return getch();
}

