# ros2-keyboard

This node uses the SDL library to grab keypresses.
To do so, it opens a window where input is received.
This window needs to becurrently focused, otherwise this node will not receive any key presses.

# Install

1. Create a new workspace.
2. `$ cd /path/to/your_ws/src`
3. Clone this repository
  - (ssh) `$ git clone git@github.com:cmower/ros2-keyboard.git`
  - (https) `$ git clone https://github.com/cmower/ros2-keyboard.git`
4. Install SDL, see [here](https://gist.github.com/cmower/5d3ad491c2acf447b7f4c307d5f88313).
5. `$ cd /path/to/your_ws`
6. `$ colcon build`

# Topics

`keydown` and `keyup` (`keyboard_msgs/Key`)
