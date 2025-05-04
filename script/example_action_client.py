#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    Simple ROS action client that connects to a "countdown" action server,
    sends a starting number, and receives periodic countdown feedback until
    completion.

Features:
  • Action Client:
      – Waits for the "countdown" server to become available.
      – Cancels any previously pending goals on startup.
      – Sends a CountdownGoal with a user‑specified starting number.
  • Callbacks:
      – active_cb: logs when the server accepts the goal.
      – feedback_cb: logs each intermediate countdown value and current action state.
      – done_cb: logs final result, prints terminal state, and cleanly shuts down the node.
  • Compatibility:
      – Designed for ROS1 with `actionlib` and a custom `ros_examples/CountdownAction`.
  • Usage:
      – Run after launching a compatible action server:
          rosrun ros_examples countdown_client.py
  • Parameters:
      – None (hardcoded starting number; modify in code or extend to pull from ROS param).

Example:
    # In one terminal, start your countdown server:
    rosrun ros_examples countdown_server.py
    # In another terminal, run this client:
    rosrun ros_examples countdown_client.py
"""

import time
import rospy
import actionlib
import ros_examples.msg


class CountdownClient():
    def __init__(self):
        # Initializes "countdown_client" node
        self.client = actionlib.SimpleActionClient("countdown", ros_examples.msg.CountdownAction)
        self.client.cancel_all_goals()

    # Waits for server to be available, then sends goal
    def send_goal(self, starting_num):
        goal_msg = ros_examples.msg.CountdownGoal()
        goal_msg.starting_num = starting_num
        rospy.loginfo(f'Starting at: {starting_num}')
        rospy.loginfo('Waiting for server...')

        if self.client.wait_for_server(rospy.Duration(5.0)):

            # self.client.send_goal_and_wait(goal_msg, rospy.Duration(0.1), rospy.Duration(1.0))
            self.client.send_goal(goal_msg,
                                  active_cb=self.goal_response_callback,
                                  feedback_cb=self.feedback_callback,
                                  done_cb=self.get_result_callback)

            rospy.loginfo("Goal sent!")
            # self.client.cancel_all_goals()

        else:
            rospy.logerr("Action server not available!")

    # Run when client accepts goal
    def goal_response_callback(self):
        rospy.loginfo('Goal accepted :)')

   # Run when client sends feedback
    def feedback_callback(self, feedback_msg):
        rospy.loginfo(f'Received feedback: {feedback_msg.current_num}: State: {self.client.get_state()}')

   # Run when client sends final result
    def get_result_callback(self, state, result):
        # PENDING    = 0  The goal has yet to be processed by the action server
        # ACTIVE     = 1  The goal is currently being processed by the action server
        # PREEMPTED  = 2  The goal received a cancel request after it started executing
        #                 and has since completed its execution (Terminal State)
        # SUCCEEDED  = 3  The goal was achieved successfully by the action server (Terminal State)
        # ABORTED    = 4  The goal was aborted during execution by the action server due
        #                 to some failure (Terminal State)
        # REJECTED   = 5  The goal was rejected by the action server without being processed,
        #                 because the goal was unattainable or invalid (Terminal State)
        # PREEMPTING = 6  The goal received a cancel request after it started executing
        #                 and has not yet completed execution
        # RECALLING  = 7  The goal received a cancel request before it started executing,
        #                 but the action server has not yet confirmed that the goal is canceled
        # RECALLED   = 8  The goal received a cancel request before it started executing
        #                 and was successfully cancelled (Terminal State)
        # LOST       = 9  An action client can determine that a goal is LOST. This should not be
        #                 sent over the wire by an action server
        rospy.loginfo(f'Result: {result.is_finished} State: {state}')
        rospy.signal_shutdown("Shutting-down client node")


def main(args=None):
   # Init ROS1 and give the node a name
    rospy.init_node("countdown_client")
    action_client = CountdownClient()

    # Sends goal and waits until it's completed
    action_client.send_goal(20)
    rospy.spin()


if __name__ == '__main__':
    main()
