#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import os

from nmea_msgs.msg import Sentence
from libnmea_navsat_driver.driver import RosNMEADriver


if __name__ == "__main__":
    rospy.init_node("log_nmea_publisher")
    in_file = "1005data.log"
    filepath = os.environ['HOME'] + "/" + in_file

    nmea_pub = rospy.Publisher("nmea_sentence", Sentence, queue_size=10)

    frame_id = RosNMEADriver.get_frame_id()

    try:
        with open(filepath, "r") as f:
            for line in f:
                data = line.strip()

                sentence = Sentence()
                sentence.header.stamp = rospy.get_rostime()
                sentence.header.frame_id = frame_id
                sentence.sentence = data

                nmea_pub.publish(sentence)

    except rospy.ROSInterruptException:
        pass
