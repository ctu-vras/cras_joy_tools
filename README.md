<!--
SPDX-License-Identifier: BSD-3-Clause
SPDX-FileCopyrightText: Czech Technical University in Prague
-->

# cras_joy_tools

Joystick and gamepad handling tools.

## Python module `cras_joy_tools.history_joystick`

Stateful gamepad/joystick library. It allows querying for button state changes, button combos etc.

API documentation is available at https://docs.ros.org/en/api/cras_joy_tools/html/index.html .

## Node `joy_translator`

The task of `joy_translator` is to "translate" the joy_node outputs so that the button/axes indices 
are the same as for a referential gamepad (which might even be virtual). This allows writing code that doesn't need to
care about which particular type of joystick (in which mode) is plugged in.

The translator is implemented in module `cras_joy_tools.joy_translator` and uses configuration files from 
`cras_joy_tools/joy_mappings` (or a custom folder).

The launch file `cras_joy_tools/launch/joy_translator.launch` has an argument called `transparent_translation`. If it is
true, the translated data are published to the `joy` topic, and the "raw" (untranslated) data are published to topic 
`joy_raw`. If `transparent_translation` is false, the `joy` topic contains the untranslated data, and topic
`joy_translated` contains the translated data.

### Gamepad translation config format

Each translation config for a gamepad should be a YAML file. If the gamepad appears under `/dev/input/by-id`, then name
the file according to this name (replacing any occurence of serial number with string `SERIAL`). If the gamepad does
not appear there, name the file as you like.

The following keys are required:

- `device_ids` (list of string): List of VID:PID strings identifying the gamepad. You can either see them in `lsusb`
  or in `/proc/bus/input/devices`.
- `num_buttons` (int): The expected number of buttons of the `Joy` messages produces by `joy_node`.
- `num_axes` (int): The expected number of axes of the `Joy` messages produces by `joy_node`.
- `deadzone` (double, optional): If set, this will be a gamepad-specific deadzone. Otherwise, a default is used.
- `buttons` (`dict[int->str]`): Mapping of input buttons and axes to the translated buttons.
- `axes` (`dict[int->str]`): Mapping of input buttons and axes to the translated axes.
- `init` (`dict[str->str]`): Expressions to evaluate when the gamepad connects.
- `callbacks` (`dict[str->str]`): Callbacks to evaluate when a new `Joy` message is received.

The values in `buttons`, `axes`, `init` and `callbacks` are expressions passed to `eval()`, so you can use a bit of
Python programming inside (e.g. math operations).

While evaluating the expressions, these variables are accessible:

* `a`: a list of raw axis values
* `b`: a list of raw button values
* `pa`: a list of raw axis values in the previous message
* `pb`: a list of raw button values in the previous message
* `vars`: a dictionary of user-defined variables (you should initialize them in the `init` section)

In section `init`, you can add key-value pairs, where the key doesn't matter (it's just a nice name), and the value is
passed to `eval()` during startup of the translator. This is nice for initializing user variables in the `vars` dict.

Section `callbacks` is similar to `init`, but the values are evaluated after each message is processed. You can e.g.
change values in the `vars` dictionary from the callbacks.

### Adding a new gamepad model

Attach the gamepad to your computer. From `ls -l /dev/input/by-id/` find out the device name (let's call it `$DEV_NAME`).
If `$DEV_NAME` contains a serial number nearby the end of it, replace the serial number with string `SERIAL`. Then
create file named `$DEV_NAME.yaml` (best by copying an existing file). If you don't see the device in `by-id`, then
give the file a name you like.

Now issue a `lsusb` in console and try to find the vendor:model pair for the gamepad. Put this ID in the `device_ids`
list in the created config file. If the gamepad is not there, you will find it in `/proc/bus/input/devices`. Search for
`js0`.

Next, edit the created YAML file. In one console run `rosrun joy joy_node`, in another one run 
`rostopic echo /joy`. Now observe the output of the `/joy` topic and try to match button/axis indices to the
expressions in the YAML file. 

## Node position_joy_cmd

Send relative positional commands by pressing gamepad buttons. This command interface is expected to work with
`cras_relative_positional_controller`.

Subscribed topics:
- `~joy` (:sensor_msgs:`Joy`): The input joystick.

Published topics:
- `~goal` (:cras_cras_relative_positional_controller:`RelativeMoveActionGoal`): Goal for the position controller.

Used actions:
- `position_controller` (:cras_cras_relative_positional_controller:`RelativeMoveAction`):
  The relative positional controller.

Parameters:
- `~use_action` (bool, default True): If true, the action interface will be used. Otherwise, just the `~goal` will be
  published.
- `~lin_dist_slow` (double, default 0.1): The distance to drive in slow mode.
- `~lin_dist_fast` (double, default 0.5): The distance to drive in fast mode.
- `~lin_vel_slow` (double, default 0.1): Slow velocity.
- `~lin_vel_fast` (double, default 0.3): Fast velocity.
- `~ang_dist_slow` (double, default 0.4): The angular distance to drive in slow mode.
- `~ang_dist_fast` (double, default 1.5): The angular distance to drive in fast mode.
- `~ang_vel_slow` (double, default 0.4): Slow angular velocity.
- `~ang_vel_fast` (double, default 0.8): Fast angular velocity.
- `~deadman_slow` (int, default 1): Index of the deadman button that needs to be pressed to execute slow motion.
- `~deadman_fast` (int, default 3): Index of the deadman button that needs to be pressed to execute fast motion.
- `~axis_linear` (int, default 7): Index of the joystick axis that defines linear motion.
- `~axis_angular` (int, default 6): Index of the joystick axis that defines the angular motion.

