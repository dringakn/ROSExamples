#!/usr/bin/env python3

"""
Author:        Dr. Ing. Ahmad Kamal Nasir
Email:         dringakn@gmail.com

Description:
    A multithreaded RadioTap packet sniffer that switches the given interface
    into monitor mode, captures 802.11 frames, and provides live statistics.

Features:
  • Monitor‑mode management:
      – Brings interface down/up
      – Sets monitor mode via iwconfig
      – Verifies interface status with ethtool/ip/iw
  • Packet capture & parsing:
      – Uses Scapy to sniff RadioTap frames
      – Counts per‑MAC address (destination, source, BSSID)
      – Extracts and tallies SSIDs from beacon frames
      – Strips payload for clean header inspection
  • Live console dashboard:
      – RX packet count and header length
      – Unique DEST, SRC, BSSID, SSID counts
      – Sorted top talkers printed every second
      – Hexdump of the most recent RadioTap header
  • Flexible filtering:
      – BPF filter on load (e.g. `not tcp or udp`, specific sender MAC)
  • Threaded design:
      – Separate threads for sniffing and dashboard printing
  • Dependencies:
      – Python3, Scapy (`sudo pip3 install scapy`)
      – Linux wireless tools (`iwconfig`, `ifconfig`, `ethtool`, `ip`)
  • Usage example:
        sudo python3 sniff_radiotap_packet.py \\
            -i wlan1 \\
            -c \\
            -s 11:22:33:44:55:66
"""

import os
import time
import argparse
import threading
from scapy.all import RadioTap, Dot11, Dot11Beacon, Dot11FCS, Dot11Elt, sniff, hexdump


class RadioTapSniffer:
    def __init__(self, iface, filter="", clear_output=False):
        self.interface = iface
        self.rx_packets = 0
        self.filter = filter
        self.clear_output = clear_output
        self.clear_cmd = 'clear' if os.name == 'posix' else 'cls'
        self.packet = RadioTap()
        self.rth = RadioTap()
        self.SSID = {}
        self.destination_addresses = {}
        self.source_addresses = {}
        self.bssid_addresses = {}
        self.stop_threads = False

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

    def sort_dictionary(self, d):
        return dict(sorted(d.items(), key=lambda item: item[1], reverse=True))

    def print_data(self):
        while not self.stop_threads:
            if self.clear_output:
                self.sys_cmd(self.clear_cmd)

            print(f"RXPKT # {self.rx_packets} [{len(self.rth)}]")
            print(f"DEST  # {len(self.destination_addresses)}")
            print(f"SRC   # {len(self.source_addresses)}")
            print(f"SSID  # {len(self.SSID)}")
            print(f"BSSID # {len(self.bssid_addresses)}")
            print(self.exec_cmd(f"ip -s link show {self.interface}\n"))
            print(f"DEST : {self.sort_dictionary(self.destination_addresses)}\n")
            print(f"SRC  : {self.sort_dictionary(self.source_addresses)}\n")
            print(f"BSSID: {self.sort_dictionary(self.bssid_addresses)}\n")
            print(f"SSID: {self.sort_dictionary(self.SSID)}\n")

            # print(self.rth)
            self.rth.show()
            # self.rth.show2()
            hexdump(self.rth)
            # print(self.rth.summary())
            # print(self.rth.command())

            time.sleep(1)

    def packet_handler(self, packet):
        self.packet = packet
        if packet.haslayer(RadioTap):
            self.rth = packet.getlayer(RadioTap)
            self.rx_packets += 1

            dest = str(packet.addr1).upper()
            self.destination_addresses[dest] = self.destination_addresses.get(dest, 0) + 1

            src = str(packet.addr2).upper()
            self.source_addresses[src] = self.source_addresses.get(src, 0) + 1

            bssid = str(packet.addr3).upper()
            self.bssid_addresses[bssid] = self.bssid_addresses.get(bssid, 0) + 1

            if packet.haslayer(Dot11Beacon):
                ssid = packet[Dot11Beacon].info.decode('utf-8', 'ignore')
                self.SSID[ssid] = self.SSID.get(ssid, 0) + 1

            self.rth.remove_payload()

    def sniff_packets(self):
        try:
            # Sniff packets in the main thread
            sniff(iface=self.interface, filter=self.filter, prn=self.packet_handler)

        except KeyboardInterrupt:
            self.stop_threads = True
            exit(0)

    def run(self):
        try:
            self.set_monitor_mode()

            # Start a separate thread for printing
            print_thread = threading.Thread(target=self.print_data)
            print_thread.start()

            # Start a separate thread for packet handling
            packet_thread = threading.Thread(target=self.sniff_packets)
            packet_thread.start()

            # Wait for the user to interrupt the program
            packet_thread.join()
            print_thread.join()

        except KeyboardInterrupt:
            self.stop_threads = True
            exit(0)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="RadioTap packet sniffer in interface monitor mode.",
                                     epilog="sudo ./sniff_radiotap_packet -i wlan1 -f 11:22:33:44:55:66")
    parser.add_argument("-i", "--iface", default="wlan1", help="Interface name")
    parser.add_argument("-c", "--clear-output", action="store_true", help="Clear screen before display")
    parser.add_argument("-s", "--src-filter_mac", default="", help="Filter message from specified MAC")
    args = parser.parse_args()

    # addr1: Destination, addr2: Source, addr3: BSSID
    if args.src_filter_mac != "":
        sniffer = RadioTapSniffer(iface=args.iface,
                                  filter=f"not(tcp or udp) and (wlan addr2 {args.src_filter_mac})",
                                  clear_output=args.clear_output)
    else:
        sniffer = RadioTapSniffer(iface=args.iface,
                                  filter=f"not(tcp or udp)",
                                  clear_output=args.clear_output)

    sniffer.run()
