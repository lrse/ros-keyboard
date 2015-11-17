## Description ##

This node uses the SDL2 library to grab keypresses. To do so, it opens a window where input is received. This window needs to be currently focused, otherwise this node will not receive any key presses. 

## Topics ##

 * ~keydown and ~keyup (keyboard/Key)

## Parameters ##

You can optionally use some parameters for customizing window appearance:

 * ~window_width, ~window_height : Window dimensions (defaults to 100x100 pixels)
 * ~window_caption : Caption to show on windows's frame (defaults to "ROS keyboard input")
 * ~shown_text : Text to show on the window (defaults to no text). This option requires SDL2's TTF extension. 
 * ~text_font : TrueType font to use; it requires the full path of the .ttf file (defaults to "/usr/share/fonts/truetype/freefont/FreeMono.ttf")
 * ~font_size : Font size (defaults to 14)
