#!/usr/bin/env python3

'''
Description:
This ROS node monitors CPU and memory usage of other ROS nodes.
Two topics are added for each node:
- /cpu_monitor/node_name/cpu
- /cpu_monitor/node_name/mem

Note:
Polling time can be configured by parameter: poll_period [1]
'''

import os
import functools
import subprocess
import psutil
from xmlrpc.client import ServerProxy  # For Python 3.x compatibility

import rosnode
import rospy
from std_msgs.msg import Float32, UInt64


def ns_join(*names):
    """Join multiple ROS names."""
    return functools.reduce(rospy.names.ns_join, names, "")


class Node:
    def __init__(self, name, pid):
        self.name = name
        self.proc = psutil.Process(pid)
        self.cpu_publisher = rospy.Publisher(f"{ns_join('~', name[1:], 'cpu')}", Float32, queue_size=20)
        self.mem_publisher = rospy.Publisher(f"{ns_join('~', name[1:], 'mem')}", UInt64, queue_size=20)

    def publish(self):
        """Publish CPU and memory usage information."""
        self.cpu_publisher.publish(Float32(self.proc.cpu_percent()))
        self.mem_publisher.publish(UInt64(self.proc.memory_info().rss))

    def alive(self):
        """Check if the node's process is running."""
        return self.proc.is_running()


def is_local_node(node_api):
    """Check if the given node is running locally."""
    this_ip = os.environ.get("ROS_IP")
    ros_ip = node_api[7:].split(':')[0]
    return "localhost" in node_api or "127.0.0.1" in node_api or (this_ip is not None and this_ip == ros_ip) or \
           subprocess.check_output("hostname").decode('utf-8').strip() in node_api


def discover_nodes(master, ignored_nodes, node_map):
    """Discover new ROS nodes."""
    for node in rosnode.get_node_names():
        if node in node_map or node in ignored_nodes:
            continue

        node_api = rosnode.get_api_uri(master, node)[2]
        if not node_api:
            rospy.logerr(f"[cpu monitor] failed to get API of node {node} ({node_api})")
            continue

        if not is_local_node(node_api):
            ignored_nodes.add(node)
            rospy.loginfo(f"[cpu monitor] ignoring node {node} with URI {node_api}")
            continue

        try:
            resp = ServerProxy(node_api).getPid('/NODEINFO')
            pid = resp[2]
            node_map[node] = Node(name=node, pid=pid)
            rospy.loginfo(f"[cpu monitor] adding new node {node}")
        except Exception as e:
            rospy.logerr(f"[cpu monitor] failed to get pid for node {node}: {e}")


def main():
    rospy.init_node("cpu_monitor")
    master = rospy.get_master()
    poll_period = rospy.get_param('~poll_period', 1.0)

    cpu_publish = rospy.Publisher("~total_cpu", Float32, queue_size=20)

    node_map = dict()
    ignored_nodes = set()
    mem_publihsers = dict()

    for key, value in psutil.virtual_memory()._asdict().items():
        mem_publihsers[key] = rospy.Publisher(f"~total_{key}_mem", Float32, queue_size=20)

    while not rospy.is_shutdown():
        discover_nodes(master, ignored_nodes, node_map)

        for node_name, node in list(node_map.items()):
            if node.alive():
                node.publish()
            else:
                rospy.logwarn(f"[cpu monitor] lost node {node_name}")
                del node_map[node_name]

        cpu_publish.publish(Float32(psutil.cpu_percent()))

        for key, value in psutil.virtual_memory()._asdict().items():
            try:
                mem_publihsers[key].publish(Float32(value))
            except Exception as ex:
                rospy.logerr(f"{ex}")

        rospy.sleep(poll_period)


if __name__ == "__main__":
    main()
