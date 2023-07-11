#!/usr/bin/env python3
"""
Author: Dr. Ing. Ahmad Kamal Nasir
Email: dringakn@gmail.com
Description: ROS Node to measure and monitor quality of WiFi link.
Example: rosrun ros_examples wireless_quality.py
Note: To install iw command (sudo apt update && sudo apt install wireless-tools)
"""

import os
import rospy
from diagnostic_msgs.msg import DiagnosticArray, KeyValue, DiagnosticStatus
from std_msgs.msg import Header


class WirelessQuality:

    def __init__(self, iface: str = "wlp0s20f3"):
        self.iface = iface
        self.tx_bytes = 0
        self.rx_bytes = 0
        self.connected_time = 0
        self.tx = 0
        self.rx = 0
        self.dt = 1

    def __parse_float(val):
        return float(val) if val != '' else 0.0

    def __parse_int(val):
        return int(float(val)) if val != '' else 0

    def __execute_command(self, cmd: str):
        result = None
        try:
            result = os.popen(cmd).read().strip()

        except Exception as ex:
            print(f"__execute_command [{cmd}]: {ex}")

        return result

    def measure_quality(self):
        header = Header()
        header.stamp = rospy.Time.now()
        array = DiagnosticArray(header=header)
        status = DiagnosticStatus(name=rospy.get_name(),
                                  hardware_id=self.iface,
                                  level=DiagnosticStatus.OK,
                                  message='')

        # Get device information
        valid_fields1 = ['addr', 'ssid', 'type', 'channel', 'txpower']
        for line in self.__execute_command(f"iw {self.iface} info").splitlines():
            key, value = line.split(maxsplit=1)
            key = key.strip()
            value = value.strip()
            if key in valid_fields1:
                status.values.append(KeyValue(key, value.strip()))

        # Get link monitoring status
        valid_fields2 = ['inactive time', 'rx bytes', 'tx bytes', 'signal',
                         'signal avg', 'tx bitrate', 'rx bitrate', 'connected time']
        for line in self.__execute_command(f"iw {self.iface} station dump").splitlines():
            key, value = line.split(':', maxsplit=1)
            key = key.strip()
            value = value.strip()
            if key in valid_fields2:
                if key == 'tx bytes':
                    temp = float(value)
                    self.tx = temp - self.tx_bytes
                    self.tx_bytes = temp
                    status.values.append(KeyValue('TX', str(self.tx)))
                    status.values.append(KeyValue('TotalTX', value))

                elif key == 'rx bytes':
                    temp = float(value)
                    self.rx = temp - self.rx_bytes
                    self.rx_bytes = temp
                    status.values.append(KeyValue('RX', str(self.rx)))
                    status.values.append(KeyValue('TotalRX', value))

                elif key == 'connected time':
                    temp = float(value.split()[0])
                    self.dt = temp - self.connected_time
                    self.connected_time = temp
                    self.dt = 1 if self.dt <= 0 else self.dt
                    status.values.append(KeyValue('DT', str(self.dt)))
                    status.values.append(
                        KeyValue('TXRate', str(self.tx/self.dt)))
                    status.values.append(
                        KeyValue('RXRate', str(self.rx/self.dt)))

                elif key == 'signal':
                    temp = float(value.split()[0])
                    status.values.append(KeyValue(key, str(temp)))

                else:
                    status.values.append(KeyValue(key, value))

        array.status.append(status)
        return array


if __name__ == '__main__':

    try:
        rospy.init_node('rssi_publisher',
                        anonymous=False,
                        log_level=rospy.INFO)
        interfacename = rospy.get_param('~interface_name', 'wlp0s20f3')
        update_rate = rospy.get_param('~update_rate_wireless_quality', 0.25)
        rospy.loginfo(
            f"Wireless quality of {interfacename} interface @ {update_rate} Hz")

        pub = rospy.Publisher("wireless_connection_info",
                              DiagnosticArray,
                              queue_size=10)

        wifi = WirelessQuality(interfacename)
        rate = rospy.Rate(update_rate)

        while not rospy.is_shutdown():

            msg = wifi.measure_quality()
            pub.publish(msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        # Catch Ctrl+C
        pass
