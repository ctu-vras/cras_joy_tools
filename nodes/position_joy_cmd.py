#!/usr/bin/env python

import rospy
from actionlib.action_client import ActionClient
from sensor_msgs.msg import Joy
from cras_joy_tools.history_joystick import HistoryJoystick
from relative_positional_controller.msg import RelativeMoveGoal, RelativeMoveAction


def cb(joy_msg):
    joy.update(joy_msg)

    if joy.is_down_any([deadman_slow, deadman_fast]) and joy.axis_touched(axis_linear):
        if use_action:
            if not pub.wait_for_server(rospy.Duration(0.2)):
                rospy.logwarn("Positional control action server not found")
                return

        slow = joy.is_down(deadman_slow)
        goal = RelativeMoveGoal()
        goal.header.stamp = rospy.Time.now()
        goal.target_x_change = joy.axes[axis_linear] * (lin_dist_slow if slow else lin_dist_fast)
        goal.linear_speed = lin_vel_slow if slow else lin_vel_fast

        rospy.loginfo("Executing relative positional command [" + str(goal.target_x_change) + ", " + str(goal.target_yaw_change) + "]")

        if use_action:
            pub.send_goal(goal)
        else:
            pub.publish(goal)

    elif joy.is_down_any([deadman_slow, deadman_fast]) and joy.axis_touched(axis_angular):
        if use_action:
            if not pub.wait_for_server(rospy.Duration(0.2)):
                rospy.logwarn("Positional control action server not found")
                return

        slow = joy.is_down(deadman_slow)
        goal = RelativeMoveGoal()
        goal.header.stamp = rospy.Time.now()
        goal.target_yaw_change = joy.axes[axis_angular] * (ang_dist_slow if slow else ang_dist_fast)
        goal.angular_speed = ang_vel_slow if slow else ang_vel_fast

        rospy.loginfo("Executing relative positional command [" + str(goal.target_x_change) + ", " + str(goal.target_yaw_change) + "]")

        if use_action:
            pub.send_goal(goal)
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
        pub = rospy.Publisher("~goal", RelativeMoveGoal, queue_size=1)

    sub = rospy.Subscriber("~joy", Joy, cb, queue_size=10)

    rospy.spin()