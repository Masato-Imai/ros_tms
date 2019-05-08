#!/usr/bin/env python
# -*- coding:utf-8 -*-

import serial

ser = serial.Serial()
ser.port = '/dev/pts/20'
ser.baudrate = 115200

ser.open()

try:
    while True:
        line = ser.readline()
        print(line)
except KeyboardInterrupt:
    ser.close()
    print("\nEnd")
