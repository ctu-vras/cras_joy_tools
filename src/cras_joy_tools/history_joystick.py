# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""
Stateful gamepad/joystick library. It allows querying for button state changes, button combos etc.

Originally implemented in nifti_teleop_joy.
"""

from sensor_msgs.msg import Joy


class HistoryJoystick(Joy):
    """Class to allow detecting transition in buttons and axes.

    It can be used like a Joy message but with additional methods."""

    def __init__(self, *args, **kwds):
        # Current state of the buttons
        super(HistoryJoystick, self).__init__(*args, **kwds)

        self.buttons = None
        # Current state of the axes
        self.axes = None
        # Header
        self.header = None

        self.old_buttons = self.old_axes = None

    def update(self, joy):
        """To be called with each new joystick data."""
        # Previous state of the buttons
        self.old_buttons = self.buttons
        # Previous state of the axes
        self.old_axes = self.axes
        self.buttons = joy.buttons
        self.axes = joy.axes
        self.header = joy.header

    def is_down(self, button_id):
        """Check if a given button is currently pressed down (state 1)."""
        return self.is_down_any((button_id,))

    def is_down_any(self, button_ids):
        """Check if any of given button is currently pressed down (state 1)."""
        try:
            res = False
            for button_id in button_ids:
                res = res or self.buttons[button_id]
            return res
        except TypeError:
            # no joystick messages yet?
            return False

    def is_down_all(self, button_ids):
        """Check if all of given button are currently pressed down (state 1)."""
        try:
            res = True
            for button_id in button_ids:
                res = res and self.buttons[button_id]
            return res
        except TypeError:
            # no joystick messages yet?
            return False

    def is_down_only(self, button_id):
        try:
            res = True
            for id in self.buttons:
                if id == button_id:
                    res = res and self.buttons[button_id]
                else:
                    res = res and not self.buttons[button_id]
            return res
        except TypeError:
            # no joystick messages yet?
            return False

    def was_down(self, button_id):
        """Check if a given button was pressed down (state 1)."""
        return self.was_down_any((button_id,))

    def was_down_any(self, button_ids):
        """Check if any of given button was pressed down (state 1)."""
        try:
            res = False
            for button_id in button_ids:
                res = res or self.old_buttons[button_id]
            return res
        except TypeError:
            # no joystick messages yet?
            return False

    def was_down_all(self, button_ids):
        """Check if all given button were pressed down (state 1)."""
        try:
            res = True
            for button_id in button_ids:
                res = res and self.old_buttons[button_id]
            return res
        except TypeError:
            # no joystick messages yet?
            return False

    def was_down_only(self, button_id):
        try:
            res = True
            for id in self.old_buttons:
                if id == button_id:
                    res = res and self.old_buttons[button_id]
                else:
                    res = res and not self.old_buttons[button_id]
            return res
        except TypeError:
            # no joystick messages yet?
            return False

    def pressed(self, button_id):
        """Check if a given button has just been pressed (transition 0->1)."""
        return self.pressed_any((button_id,))

    def pressed_any(self, button_ids):
        """Check if at least one in a given list of buttons has just been pressed (transition 0->1)."""
        try:
            res = False
            for button_id in button_ids:
                res = res or (self.buttons[button_id] and not self.old_buttons[button_id])
            return res
        except (AttributeError, TypeError):
            # not enough joystick messages yet?
            return False

    def pressed_button_combo(self, button_ids):
        """Check if the given button combination has just been pressed (
        all buttons 1 and one of them just transitioned 0->1)"""
        return self.is_down_all(button_ids) and not self.was_down_all(button_ids) and self.pressed_any(button_ids)

    def released(self, button_id):
        """Check if a given button has just been released (transition 1->0)."""
        return self.released_any((button_id,))

    def released_any(self, button_ids):
        """Check if any of given buttons has just been released (transition 1->0)."""
        try:
            res = False
            for button_id in button_ids:
                res = res or (not self.buttons[button_id] and self.old_buttons[button_id])
            return res
        except (AttributeError, TypeError):
            # not enough joystick messages yet?
            return False

    def released_all(self, button_ids):
        """Check if all given buttons have just been released (transition 1->0)."""
        try:
            return not self.is_down_any(button_ids) and self.was_down_any(button_ids)
        except (AttributeError, TypeError):
            # not enough joystick messages yet?
            return False

    def button_changed(self, button_id):
        """Check if a given button state has just changed (either transitions)."""
        return self.button_changed_any((button_id,))

    def button_changed_any(self, button_ids):
        """Check if any of given buttons state has just changed (either transitions)."""
        try:
            res = False
            for button_id in button_ids:
                res = res or (self.buttons[button_id] != self.old_buttons[button_id])
            return res
        except (TypeError, AttributeError):
            # not enough joystick messages yet?
            return False

    def axis_moved(self, axis_id):
        """Check if a given axis has moved."""
        try:
            return self.axes[axis_id] != self.old_axes[axis_id]
        except (TypeError, AttributeError):
            # no joystick messages yet?
            return False

    def axis_touched(self, axis_id):
        """Check if a given axis that was released has moved (transition
        0->anything else)."""
        try:
            return self.axes[axis_id] and not self.old_axes[axis_id]
        except (TypeError, AttributeError):
            # no joystick messages yet?
            return False

    def axis_released(self, axis_id):
        """Check if a given axis has just been released has (transition anything
        else->0)."""
        try:
            return not self.axes[axis_id] and self.old_axes[axis_id]
        except (TypeError, AttributeError):
            # no joystick messages yet?
            return False
