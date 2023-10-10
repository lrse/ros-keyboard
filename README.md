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
4. Install SDL 1.2 with the command `sudo apt install libsdl1.2-dev`.
5. `$ cd /path/to/your_ws`
6. `$ colcon build`

# Nodes

## `keyboard`

Publishes keyboard events when the window is focused.

* Published topics: `keydown` and `keyup` (`keyboard_msgs/Key`)
* Parameters
  * `allow_repeat` (`bool`): Enables or disables the keyboard repeat rate. Default is `false`.
  * `repeat_delay` (`int`): How long the key must be pressed before it begins repeating. Default is `SDL_DEFAULT_REPEAT_DELAY`.
  * `repeat_interval` (`int`): Key replay speed. Default is `SDL_DEFAULT_REPEAT_INTERVAL`.

## `keyboard_to_joy.py`

Converts keyboard events to a joy message.

* Published topic: `joy` (`sensor_msgs/Joy`)
* Subscribed topics: `keydown` and `keyup` (`keyboard_msgs/Key`)
* Parameters
  * `config_file_name` (`str`): File name for the configuration file. See [here](keyboard/config/example_config.yaml) for an example.
  * `sampling_frequency` (`int`): Rate (Hz) at which `joy` messages are published. Default is `50`.

### Example:

In one terminal, start the `keyboard` node.
```
$ ros2 run keyboard keyboard
```

In a second terminal, start the `keyboard_to_joy.py` node.
```
$ ros2 run keyboard keyboard_to_joy.py --ros-args -p config_file_name:=/path/to/keyboard/config/example_config.yaml
```

Remember to replace `/path/to` with the path to where you cloned the `keyboard` repository.
