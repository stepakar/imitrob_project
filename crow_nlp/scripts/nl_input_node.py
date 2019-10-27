#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Karla Stepanova, Zdenek Kasner, Jan Behrens
@mail: karla.stepanova@cvut.cz
"""
from crow_nlp.msg import SentenceProgram
import rospy

class nl_input_node():
    # listens to /averaged_markers from object_detection package and in parallel to button presses. When there is an approval
    # to collect cubes (key a -> y), robot picks up the currently detected cubes and places them to storage
    def __init__(self):
        rospy.init_node('nl_input_node', anonymous=False)

        input_sentences = [
            "Pick a red cube",
            #"Take any cube",
            "Glue a point here",
            "Glue a point on position 4 3",
            "Glue a point in the center of a panel",
            "Glue a point here and glue a point on position 4 3",
            #"Glue a point in the center of the blue panel",
            "Pick a red cube",
            "Pick a red cube and pick a green cube"
            #"Put a red cube on position 0 0",
            #"Take any cube",
             #"Put it down",
            #"Learn a new task called glue a point on the panel",
            #"Take a glue. Glue a point in the center of the panel. Put the glue on position three five.",
            #"Glue a point on the panel",
            #"Glue a point in the center of the panel and glue a point on the right side of the panel"
            #'Pick a red cube'
            #"First make a point here and then make a point here"
        ]

        # input_sentences = [
        #     "Define a new area called storage 1 with corners 0 0, 0 2, 2 2, 2 0",
        #     "Glue a point in the center of storage 1",
        #     "Learn to build a tower called tower 1",
        #     "Build tower 1",
        #     #"Show actions learned from demonstration",
        #     "Glue a point here",
        #     #"Glue a point in the center of the panel",
        #     #"Glue a point on the right side of the panel",
        #     #"Pick a red cube",
        #     #"Put it on top of the green cube.",
        #     #"Take any cube",
        #     #"Put it down",
        #     #"Learn a new task called glue a point on the panel",
        #     #"Take a glue. Use it to glue a point in the center of the panel. Put the glue on position three five.",
        #     #"Glue a point on the panel and take any cube."
        # ]

        self.pub = rospy.Publisher('/nl_input', SentenceProgram, queue_size=10)
        rospy.sleep(1.0)
        msg = SentenceProgram()
        msg.header.stamp = rospy.Time.now()
        msg.data = input_sentences
        self.pub.publish(msg)

if __name__ == '__main__':
        nl_input_node()
        # spin() simply keeps python from exiting until this node is stopped
