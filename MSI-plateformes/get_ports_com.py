#!/usr/bin/env python3
import serial 
import serial.tools.list_ports



def get_com_ports():
    ports = serial.tools.list_ports.comports()

    for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))


if __name__ == "__main__":
    get_com_ports()
 
