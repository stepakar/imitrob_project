#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Gabriela Sejnova, Karla Stepanova
@mail: karla.stepanova@cvut.cz
"""
from crow_nlp.msg import SentenceProgram
import rospy
from asr.msg import *
import time

class nl_input_node():
    # listens to /averaged_markers from object_detection package and in parallel to button presses. When there is an approval
    # to collect cubes (key a -> y), robot picks up the currently detected cubes and places them to
    #### Subscriber reads speech transcripts published by /asr_output and once there is silence again, it publishes them as list
    def __init__(self):
        rospy.init_node('nl_input_node', anonymous=False)
        self.sentences = []
        self.whole = False
        self.sub = rospy.Subscriber('/asr_output', asr_output, queue_size=1, callback=self.callback)
        self.pub = rospy.Publisher('/nl_input', SentenceProgram, queue_size=10)
        print("Waiting for speech data")
        while True:
            while not self.sentences:
                time.sleep(.5)
            rospy.sleep(1.0)
            msg = SentenceProgram()
            msg.header.stamp = rospy.Time.now()
            msg.data = self.sentences
            print("Publishing: {}".format(msg.data))
            self.pub.publish(msg)
            self.whole = False
            self.sentences = []

    def callback(self, data):
        self.sentences.append(data.data)
        return


if __name__ == '__main__':
        nl_input_node()
        # spin() simply keeps python from exiting until this node is stopped
