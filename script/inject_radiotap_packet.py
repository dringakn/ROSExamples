#!/usr/bin/env python3
"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    A simple RadioTap packet injector that puts a given Wi‑Fi interface into monitor mode,
    crafts a raw 802.11 frame with Radiotap header, and injects it at a configurable rate.

Features:
  • Automatically discovers all “wl*” interfaces and their MAC addresses  
  • Switches your chosen interface into monitor mode  
  • Builds a Radiotap header with Flags, Rate, Channel, Antenna signal, etc.  
  • Crafts a Dot11 data frame (type 2/subtype 0), with configurable source, destination, BSSID  
  • Sends arbitrary‑length payloads (default 1440 bytes of “F”)  
  • Adjustable packet count and inter‑packet interval  
  • Console output of scan/change commands for debugging  

Dependencies:
  – Python packages: scapy, netifaces, argparse  
  – System tools: ifconfig, iwconfig, iw, ethtool, ip  
  – Must run as root (monitor mode + raw injection)  

Usage Example:
    sudo ./inject_radiotap_packet.py \
        --iface wlan1 \
        --dst-mac 11:22:33:44:55:66 \
        --num-packets 100 \
        --freq 5

"""

import os
import netifaces
import argparse
from scapy.all import RadioTap, Dot11, Dot11Beacon, Dot11Elt, sendp


class RadioTapInjector:

    def __init__(self, iface, dst_mac='', clear_output=False, num_packets=1, freq=1):
        print(self.get_wifi_mac_addresses())
        self.interface = iface
        self.rx_packets = 0
        self.interval = 1.0/max(int(freq), 1)
        self.src_mac = "11:22:33:44:55:66"
        self.dst_mac = "FF:FF:FF:FF:FF:FF" if dst_mac == '' else dst_mac
        self.bssid = self.get_wifi_mac_addresses()[iface]
        self.clear_output = clear_output
        self.num_packets = int(num_packets)
        self.clear_cmd = 'clear' if os.name == 'posix' else 'cls'

        self.rth = RadioTap(
            len=18,
            present=0x0000482e,  # 'Flags+Rate+Channel+dBm_AntSignal+Antenna'
            Flags=0x10,             # Flags as per the provided information
            Rate=54,               # Data Rate: 27,0 Mb/s
            # Channel=2412,  # Channel frequency: 2412 [BG 1]
            ChannelFrequency=2412,  # Channel frequency: 2412 [BG 1]
            ChannelFlags='OFDM+2GHz',  # Channel Frequency
            dBm_AntSignal=-35,      # Antenna signal: -XdBm
            Antenna=0,              # Antenna: 0
            RXFlags=0               # RXFlags: 0
        )

        # addr1: Destination, addr2: Source, addr3: BSSID
        self.dot11 = Dot11(type=2,
                           subtype=0,
                           addr1=self.dst_mac,
                           addr2=self.src_mac,
                           addr3=self.bssid)
        self.payload = 'F'*1440
        self.packet = self.rth / self.dot11 / self.payload

    def get_wifi_mac_addresses(self):
        wifi_interfaces = [interface for interface in netifaces.interfaces() if "wl" in interface.lower()]

        wifi_mac_addresses = {}
        for wifi_interface in wifi_interfaces:
            try:
                mac_address = netifaces.ifaddresses(wifi_interface)[netifaces.AF_LINK][0]['addr']
                wifi_mac_addresses[wifi_interface] = mac_address.upper()
            except (KeyError, IndexError):
                wifi_mac_addresses[wifi_interface] = "N/A"

        return wifi_mac_addresses

    def exec_cmd(self, cmd):
        return os.popen(cmd).read()

    def sys_cmd(self, cmd):
        return os.system(cmd)

    def set_monitor_mode(self):
        self.exec_cmd(f"ifconfig {self.interface} down")
        self.exec_cmd(f"iwconfig {self.interface} mode monitor")
        self.exec_cmd(f"ethtool -i {self.interface}")
        self.exec_cmd(f"ip -s link show {self.interface}")
        self.exec_cmd(f"iw {self.interface} info")
        self.exec_cmd(f"ifconfig {self.interface} up")

    def run(self):
        try:
            self.set_monitor_mode()
            sendp(self.packet, iface=self.interface, inter=self.interval,
                  loop=False, count=self.num_packets, verbose=True)
        except KeyboardInterrupt:
            exit(0)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="RadioTap packet injector in interface monitor mode.",
                                     epilog="sudo ./inject_radiotap_packet -i wlan1 -f 11:22:33:44:55:66")
    parser.add_argument("-i", "--iface", default="wlan1", help="Interface name")
    parser.add_argument("-n", "--num-packets", default="1", help="Number of packets to transmit [-1=Infinite]")
    parser.add_argument("-f", "--freq", default="1", help="Packet publishing frequency")
    parser.add_argument("-c", "--clear-output", action="store_true", help="Clear screen before display")
    parser.add_argument("-d", "--dst-mac", default="FF:FF:FF:FF:FF:FF", help="Destination MAC Address")
    args = parser.parse_args()

    sniffer = RadioTapInjector(iface=args.iface,
                               dst_mac=args.dst_mac,
                               clear_output=args.clear_output,
                               num_packets=args.num_packets,
                               freq=args.freq)

    sniffer.run()
