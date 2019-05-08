#!/usr/bin/env python
# -*- coding:utf-8 -*-

import serial
import time

ser = serial.Serial()
ser.port = '/dev/pts/5'
ser.baudrate = 115200

ser.open()
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