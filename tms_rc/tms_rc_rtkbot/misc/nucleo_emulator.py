#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys
import serial
import time

ser = serial.Serial()
ser.baudrate = 115200
ser.port = '/dev/pts/'

if(len(sys.argv) == 2):
    ser.port += sys.argv[1]
else:
    print("[Error]: Expected Port Number")
    sys.exit(1)

ser.open()
print("Opened " + ser.port)
t1 = time.time()

try:
    while True:
        time.sleep(0.01) # 100Hz
        dr = 0
        dl = 0
        t2 = time.time()
        dt = t2 - t1

        ser.write("{},{},{},\n".format(dr, dl, dt))
        t1 = t2

except KeyboardInterrupt:
    ser.close()
    print("\nEnd")
