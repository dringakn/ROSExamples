#!/usr/bin/env python3

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
