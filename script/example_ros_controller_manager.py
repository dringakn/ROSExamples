#!/usr/bin/env python3

import rospy
from controller_manager_msgs.srv import ReloadControllerLibraries, ListControllerTypes
from controller_manager_msgs.srv import ListControllers, LoadController, UnloadController, SwitchController


class ControllerManagerClient:
    def __init__(self):
        rospy.init_node('controller_manager_client')
        rospy.wait_for_service('/controller_manager/reload_controller_libraries', rospy.Duration(5).to_sec())
        rospy.wait_for_service('/controller_manager/list_controller_types', rospy.Duration(5).to_sec())
        rospy.wait_for_service('/controller_manager/list_controllers', rospy.Duration(5).to_sec())
        rospy.wait_for_service('/controller_manager/load_controller', rospy.Duration(5).to_sec())
        rospy.wait_for_service('/controller_manager/unload_controller', rospy.Duration(5).to_sec())
        rospy.wait_for_service('/controller_manager/switch_controller', rospy.Duration(5).to_sec())
        self.reload_libraries = rospy.ServiceProxy('/controller_manager/reload_controller_libraries', ReloadControllerLibraries)
        self.list_controller_types = rospy.ServiceProxy('/controller_manager/list_controller_types', ListControllerTypes)
        self.list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
        self.load_controller = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
        self.unload_controller = rospy.ServiceProxy('/controller_manager/unload_controller', UnloadController)
        self.switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

    def reload_controller_libraries(self):
        try:
            self.reload_libraries()
            rospy.loginfo("Controller libraries reloaded successfully.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to reload controller libraries: {e}")

    def list_all_controller_types(self):
        try:
            controller_types = self.list_controller_types()
            rospy.loginfo(f"Controller Type:")
            for idx, ctype in enumerate(controller_types.types):
                rospy.loginfo(f"[{idx+1}] {ctype} [{controller_types.base_classes[idx]}]")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to list controller types: {e}")

    def list_all_controllers(self):
        try:
            rospy.loginfo(f"Controller State:")
            controllers = self.list_controllers()
            for idx, controller in enumerate(controllers.controller):
                rospy.loginfo(f"[{idx+1}] {controller.name}[{controller.type}] is {controller.state}: {[r.hardware_interface for r in controller.claimed_resources]}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to list controllers: {e}")

    def load_controller(self, name, type, joints=[]):
        try:
            self.load_controller(name, type, joints)
            rospy.loginfo(f"Controller '{name}' loaded successfully.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to load controller '{name}': {e}")

    def unload_controller(self, name):
        try:
            self.unload_controller(name)
            rospy.loginfo(f"Controller '{name}' unloaded successfully.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to unload controller '{name}': {e}")

    def switch_controller(self, start_controllers=[], stop_controllers=[], strictness=2):
        try:
            self.switch_controller(start_controllers, stop_controllers, strictness)
            rospy.loginfo("Switched controllers successfully.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to switch controllers: {e}")

    def is_controller_running(self, controller_name):
        try:
            controllers = self.list_controllers()
            for controller in controllers.controller:
                if controller.name == controller_name:
                    if controller.state == "running":
                        rospy.loginfo(f"Controller {controller_name} is running.")
                        return True
                    else:
                        rospy.loginfo(f"Controller {controller_name} is not running. State: {controller.state}")
                        return False
            rospy.logwarn(f"Controller {controller_name} not found.")
            return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to list controllers: {e}")
            return False

    def run_controller(self, controller_name):
        try:
            controllers = self.list_controllers()
            for controller in controllers.controller:
                if controller.name == controller_name:
                    if controller.state == "initialized":
                        rospy.loginfo(f"Starting controller {controller_name}...")
                        self.switch_controller([controller_name], [], 2)  # Start the controller
                        rospy.loginfo(f"Controller {controller_name} started.")
                    elif controller.state == "running":
                        rospy.loginfo(f"Controller {controller_name} is already running.")
                    else:
                        rospy.logwarn(f"Controller {controller_name} is not in an appropriate state to be started.")
                    return
            rospy.logwarn(f"Controller {controller_name} not found.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to start controller {controller_name}: {e}")


if __name__ == '__main__':
    controller_manager = ControllerManagerClient()
    controller_manager.reload_controller_libraries()
    controller_manager.list_all_controller_types()
    controller_manager.list_all_controllers()

    # Check if the controller is running
    if not controller_manager.is_controller_running('joint_based_cartesian_traj_controller'):
        # Load the controller if it's not running
        controller_manager.load_controller('joint_based_cartesian_traj_controller')
        # Run the controller if it's in the initialized state
        controller_manager.run_controller('joint_based_cartesian_traj_controller')

    # Example usage:
    # controller_manager.load_controller("my_controller", "MyControllerType")
    # controller_manager.unload_controller("my_controller")
    # controller_manager.switch_controller(["new_controller"], ["old_controller"])
