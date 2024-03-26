#!/usr/bin/env python3

import sys

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
import geometry_msgs.msg as geometry_msgs

from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]


class TrajectoryClient:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):
        rospy.init_node("ur10e_test_controllers")

        self.switch_srv = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)

        try:
            self.switch_srv.wait_for_service(rospy.Duration(5).to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr(f"Could not reach controller switch service. Msg: {err}")
            sys.exit(-1)

        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[1]

    def send_joint_trajectory(self):

        self.switch_controller(self.joint_trajectory_controller)
        trajectory_client = actionlib.SimpleActionClient(f"{self.joint_trajectory_controller}/follow_joint_trajectory", FollowJointTrajectoryAction)

        if not trajectory_client.wait_for_server(rospy.Duration(5)):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        position_list = [[0.0, -1.57, -1.57, 0, 0, 0]]
        position_list.append([0.2, -1.57, -1.57, 0, 0, 0])
        position_list.append([-0.5, -1.57, -1.2, 0, 0, 0])

        velocity_list = [[0.2, 0, 0, 0, 0, 0]]
        velocity_list.append([-0.2, 0, 0, 0, 0, 0])
        velocity_list.append([0.0, 0, 0, 0, 0, 0])

        duration_list = [3.0, 7.0, 10.0]

        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.velocities = velocity_list[i]
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        rospy.loginfo(f"Executing trajectory using the {self.joint_trajectory_controller}")

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo(f"Trajectory execution finished in state {result.error_code}")

    def send_cartesian_trajectory(self):
        self.switch_controller(self.cartesian_trajectory_controller)

        goal = FollowCartesianTrajectoryGoal()
        trajectory_client = actionlib.SimpleActionClient(f"{self.cartesian_trajectory_controller}/follow_cartesian_trajectory", FollowCartesianTrajectoryAction)

        if not trajectory_client.wait_for_server(rospy.Duration(5)):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        pose_list = [
            geometry_msgs.Pose(geometry_msgs.Vector3(0.4, -0.1, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)),
            geometry_msgs.Pose(geometry_msgs.Vector3(0.4, -0.1, 0.6), geometry_msgs.Quaternion(0, 0, 0, 1)),
            geometry_msgs.Pose(geometry_msgs.Vector3(0.4, 0.3, 0.6), geometry_msgs.Quaternion(0, 0, 0, 1)),
            geometry_msgs.Pose(geometry_msgs.Vector3(0.4, 0.3, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)),
            geometry_msgs.Pose(geometry_msgs.Vector3(0.4, -0.1, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)),
        ]

        duration_list = [3.0, 4.0, 5.0, 6.0, 7.0]
        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        rospy.loginfo(f"Executing trajectory using the {self.cartesian_trajectory_controller}")
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()

        rospy.loginfo(f"Trajectory execution finished in state {result.error_code}")

    def switch_controller(self, target_controller):
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)


if __name__ == "__main__":

    client = TrajectoryClient()

    trajectory_type = "cartesian"
    if trajectory_type == "joint_based":
        client.send_joint_trajectory()
    elif trajectory_type == "cartesian":
        client.send_cartesian_trajectory()
