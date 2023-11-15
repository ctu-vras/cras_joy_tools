#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague


"""
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
"""


import rospy

from actionlib.action_client import ActionClient, GoalID
from cras_joy_tools.history_joystick import HistoryJoystick
from sensor_msgs.msg import Joy

try:
    from cras_relative_positional_controller.msg import RelativeMoveActionGoal, RelativeMoveAction
except ImportError:
    # Backwards compatibility before the package was renamed
    from relative_positional_controller.msg import RelativeMoveActionGoal, RelativeMoveAction


g_goal_id = 0


def generate_id():
    global g_goal_id
    id, g_goal_id = g_goal_id, g_goal_id + 1
    now = rospy.Time.now()
    return GoalID(id="%s-%i-%.3f" % (rospy.get_caller_id(), id, now.to_sec()), stamp=now)


def cb(joy_msg):
    joy.update(joy_msg)

    if joy.is_down_any([deadman_slow, deadman_fast]) and joy.axis_touched(axis_linear):
        if use_action:
            if not pub.wait_for_server(rospy.Duration(0.2)):
                rospy.logwarn("Positional control action server not found")
                return

        slow = joy.is_down(deadman_slow)
        goal = RelativeMoveActionGoal()
        goal.goal.header.stamp = rospy.Time.now()
        goal.goal_id = generate_id()
        goal.goal.target_x_change = joy.axes[axis_linear] * (lin_dist_slow if slow else lin_dist_fast)
        goal.goal.linear_speed = lin_vel_slow if slow else lin_vel_fast

        rospy.loginfo("Executing relative positional command [" +
                      str(goal.goal.target_x_change) + ", " + str(goal.goal.target_yaw_change) + "]")

        if use_action:
            pub.send_goal(goal.goal)
        else:
            pub.publish(goal)

    elif joy.is_down_any([deadman_slow, deadman_fast]) and joy.axis_touched(axis_angular):
        if use_action:
            if not pub.wait_for_server(rospy.Duration(0.2)):
                rospy.logwarn("Positional control action server not found")
                return

        slow = joy.is_down(deadman_slow)
        goal = RelativeMoveActionGoal()
        goal.goal.header.stamp = rospy.Time.now()
        goal.goal_id = generate_id()
        goal.goal.target_yaw_change = joy.axes[axis_angular] * (ang_dist_slow if slow else ang_dist_fast)
        goal.goal.angular_speed = ang_vel_slow if slow else ang_vel_fast

        rospy.loginfo("Executing relative positional command [" +
                      str(goal.goal.target_x_change) + ", " + str(goal.goal.target_yaw_change) + "]")

        if use_action:
            pub.send_goal(goal.goal)
        else:
            pub.publish(goal)


if __name__ == '__main__':
    rospy.init_node("position_joy_cmd")

    use_action = rospy.get_param("~use_action", True)
    lin_dist_slow = rospy.get_param("~lin_dist_slow", 0.1)
    lin_dist_fast = rospy.get_param("~lin_dist_fast", 0.5)
    lin_vel_slow = rospy.get_param("~lin_vel_slow", 0.1)
    lin_vel_fast = rospy.get_param("~lin_vel_fast", 0.3)

    ang_dist_slow = rospy.get_param("~ang_dist_slow", 0.4)
    ang_dist_fast = rospy.get_param("~ang_dist_fast", 1.5)
    ang_vel_slow = rospy.get_param("~ang_vel_slow", 0.4)
    ang_vel_fast = rospy.get_param("~ang_vel_fast", 0.8)

    deadman_slow = rospy.get_param("~deadman_slow", 1)
    deadman_fast = rospy.get_param("~deadman_fast", 3)
    axis_linear = rospy.get_param("~axis_linear", 7)
    axis_angular = rospy.get_param("~axis_angular", 6)

    joy = HistoryJoystick()

    if use_action:
        pub = ActionClient("position_controller", RelativeMoveAction)
    else:
        pub = rospy.Publisher("~goal", RelativeMoveActionGoal, queue_size=1)

    sub = rospy.Subscriber("~joy", Joy, cb, queue_size=10)

    rospy.spin()
