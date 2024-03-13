#!/usr/bin/env python3

import rospy
from controller_manager_msgs.srv import ReloadControllerLibraries, ListControllerTypes
from controller_manager_msgs.srv import ListControllers, LoadController, UnloadController, SwitchController
from controller_manager_msgs.srv import SwitchControllerRequest, LoadControllerRequest, UnloadControllerRequest


class ControllerManagerClient:
    def __init__(self):
        rospy.init_node('controller_manager_client')
        rospy.wait_for_service('/controller_manager/reload_controller_libraries', rospy.Duration(5).to_sec())
        rospy.wait_for_service('/controller_manager/list_controller_types', rospy.Duration(5).to_sec())
        rospy.wait_for_service('/controller_manager/list_controllers', rospy.Duration(5).to_sec())
        rospy.wait_for_service('/controller_manager/load_controller', rospy.Duration(5).to_sec())
        rospy.wait_for_service('/controller_manager/unload_controller', rospy.Duration(5).to_sec())
        rospy.wait_for_service('/controller_manager/switch_controller', rospy.Duration(5).to_sec())

        self.reload_libraries_service = rospy.ServiceProxy('/controller_manager/reload_controller_libraries', ReloadControllerLibraries)
        self.list_controller_types_service = rospy.ServiceProxy('/controller_manager/list_controller_types', ListControllerTypes)
        self.list_controllers_service = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
        self.load_controller_service = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
        self.unload_controller_service = rospy.ServiceProxy('/controller_manager/unload_controller', UnloadController)
        self.switch_controller_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

        self.controller_types = {}
        self.controllers = {}

    def reload_controller_libraries(self):
        try:
            self.reload_libraries_service()
            rospy.loginfo("Controller libraries reloaded successfully.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to reload controller libraries: {e}")

    def get_all_controller_types(self):
        try:
            rospy.loginfo(f"Controller Type:")
            res = self.list_controller_types_service()
            self.controller_types = {k: v for k, v in zip(res.types, res.base_classes)}
            idx = 1
            for key, value in self.controller_types.items():
                rospy.loginfo(f"[{idx}] {key} [{value}]")
                idx += 1
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to list controller types: {e}")

    def get_all_controllers(self):
        try:
            rospy.loginfo(f"Controller State:")
            res = self.list_controllers_service()
            for idx, controller in enumerate(res.controller):
                self.controllers[controller.name] = controller
                rospy.loginfo(f"[{idx+1}] {controller.name}[{controller.type}] is {controller.state}: {[r.hardware_interface for r in controller.claimed_resources]}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to list controllers: {e}")

    def load_controller(self, name):
        try:
            req = LoadControllerRequest()
            req.name = name
            res = self.load_controller_service(req)
            if res.ok:
                rospy.loginfo(f"Controller '{name}' loading and initialization successfully.")
            else:
                rospy.loginfo(f"Controller '{name}' loading and initialization failed. is it already loaded?")

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to load controller '{name}': {e}")

    def unload_controller(self, name):
        try:
            req = UnloadControllerRequest()
            req.name = name
            res = self.unload_controller_service(req)
            if res.ok:
                rospy.loginfo(f"Controller '{name}' unloaded successfully.")
            else:
                rospy.loginfo(f"Controller '{name}' unloading failed. is it already unloaded?")

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to load controller '{name}': {e}")

    def switch_controller(self, start_controllers=[], stop_controllers=[], strictness=1):
        """ start/stop controller(s)
        Args:
            start_controllers (list, optional): List of controllers to start. Defaults to [].
            stop_controllers (list, optional): List of controllers to stop. Defaults to [].
            strictness (int, optional): Strictness of the switch, 1=BEST-EFFORT, 2=STRICT. Defaults to 2.
        """
        try:
            req = SwitchControllerRequest()
            req.start_controllers = start_controllers
            req.stop_controllers = stop_controllers
            req.strictness = strictness
            # start the controllers as soon as their hardware dependencies are ready, will wait for all interfaces to be ready otherwise the timeout in seconds before aborting pending controllers. 0 for infinite timeout.
            req.start_asap = True
            req.timeout = 0
            res = self.switch_controller_service(req)
            if res.ok:
                rospy.loginfo(f"Switched controllers successfully.")
            else:
                rospy.loginfo(f"Switching controllers failed. Is the hardware interface ready?")

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to switch controllers: {e}")

    def start_controller(self, name):
        try:
            if name not in self.controllers.keys():
                rospy.logwarn(f"Controller '{name}' is not loaded")
                return

            if self.controllers[name].state == "running":
                rospy.loginfo(f"Controller '{name}' is already running.")
            else:
                self.switch_controller(start_controllers=[name], stop_controllers=[], strictness=1)

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to start controller '{name}': {e}")

    def stop_controller(self, name):
        try:
            if name not in self.controllers.keys():
                rospy.logwarn(f"Controller '{name}' is not loaded")
                return

            if self.controllers[name].state == "running":
                self.switch_controller(start_controllers=[], stop_controllers=[name], strictness=1)
            else:
                rospy.loginfo(f"Controller '{name}' is not running: {self.controllers[name].state}.")

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to stop controller '{name}': {e}")

    def controller_status(self, name):
        try:
            if name not in self.controllers.keys():
                rospy.logwarn(f"Controller '{name}' is not loaded")
                return

            rospy.loginfo(f"{name} status: ")
            print(self.controllers[name])

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get controller '{name}' status: {e}")


if __name__ == '__main__':
    controller_manager = ControllerManagerClient()
    controller_manager.reload_controller_libraries()
    controller_manager.get_all_controller_types()
    controller_manager.get_all_controllers()

    # controller_manager.load_controller('joint_based_cartesian_traj_controller')
    controller_manager.start_controller('joint_based_cartesian_traj_controller')
    controller_manager.stop_controller('joint_based_cartesian_traj_controller')
    controller_manager.controller_status('joint_based_cartesian_traj_controller')

    # controller_manager.unload_controller('joint_based_cartesian_traj_controller')
