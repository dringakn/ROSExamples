#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    A ROS Action Server called “countdown” that takes a starting integer and
    counts down to zero at 1 Hz, publishing feedback on each tick. It cleanly
    handles goal completion, preemption, and abortion.

Features:
  • Action name: “countdown”
  • Action type: ros_examples/CountdownAction
  • Publishes CountdownFeedback (current_num) at 1 Hz
  • On reaching zero, sets succeeded with CountdownResult.is_finished = True
  • Supports preemption: if client cancels, sends preempted with is_finished = False
  • On unexpected shutdown while active, sends aborted with is_finished = False
  • Logs state transitions (start, feedback, success, preempt, abort)

Usage:
    # Start the server:
    rosrun your_package countdown_server.py

    # From a client, send a goal:
    rostopic pub /countdown/goal ros_examples/CountdownActionGoal \
      '{goal: {starting_num: 10}}'

Dependencies:
  • rospy
  • actionlib
  • ros_examples (defines CountdownAction, CountdownFeedback, CountdownResult)
"""

import time
import rospy
import actionlib
import ros_examples.msg


class CountdownServer():
    # Create Feedback and Result messages
    def __init__(self):
        # Create the server
        self.server = actionlib.SimpleActionServer('countdown',
                                                   ros_examples.msg.CountdownAction,
                                                   self.execute_callback,
                                                   False)

        # Start the server
        self.server.start()
        rospy.loginfo("Starting Action Server")

    def send_feedback(self, number):
        feedback_msg = ros_examples.msg.CountdownFeedback()
        feedback_msg.current_num = number
        self.server.publish_feedback(feedback_msg)

    def send_result_success(self):
        rospy.loginfo('Success!')
        result = ros_examples.msg.CountdownResult()
        result.is_finished = True
        self.server.set_succeeded(result)

    def send_preempted(self):
        rospy.loginfo('Preempt requested.')
        result = ros_examples.msg.CountdownResult()
        result.is_finished = False
        self.server.set_preempted(result, "Preempted!")

    def send_aborted(self):
        rospy.loginfo('Abort requested.')
        result = ros_examples.msg.CountdownResult()
        result.is_finished = False
        self.server.set_aborted(result, "Aborted!")

    # Callback function to run after acknowledging a goal from the client
    def execute_callback(self, goal_handle):
        rospy.loginfo("Starting countdown…")

        # Initiate the feedback message's current_num as the action request's starting_num
        starting_num = goal_handle.starting_num
        rate = rospy.Rate(1)  # publish once per second
        while rospy.is_shutdown() is False:
            self.send_feedback(starting_num)
            rospy.loginfo(f'Feedback: {starting_num}')
            starting_num -= 1

            if starting_num == 0:
                self.send_result_success()
                break

            elif self.server.is_preempt_requested():
                self.send_preempted()
                break

            rate.sleep()

        # Shutdown called when countdown is in progress
        if self.server.is_active():
            self.send_aborted()


def main(args=None):
    # Init ROS1 and give the node a name
    rospy.init_node("countdown_server")
    countdown_server = CountdownServer()
    rospy.spin()


if __name__ == '__main__':
    main()
