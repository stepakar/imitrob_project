#!/usr/bin/env python
import rospy
from asr.msg import *
from asr.srv import *

## speech recognition running in time loop (s) and language ("en-US" or "cs-CZ")
## @param time int: seconds per loop
## @param lang str: language
def listening_client(time, lang):
    rospy.wait_for_service('asr')
    try:
        asr = rospy.ServiceProxy('asr', SpeechReco)     
        print("Calling ASR service..")
        response = asr(time, lang)
        return response.str

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    answer = listening_client(time=3, lang="en-US")
    print("Service returned: {}".format(answer))
