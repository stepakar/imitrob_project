#!/usr/bin/env python3
import rospy
import speech_recognition as sr
from asr.msg import asr_output
from asr.srv import *


def speech_reco(req):
    time = req.time
    lang = req.lang
    rec = sr.Recognizer()
    while True:
        try:
            with sr.Microphone() as source:
                print("Speak now:")
                audio = rec.listen(source, phrase_time_limit=time)
                recognized = rec.recognize_google(audio, language=lang)
                print ("Recognized: {}".format(recognized))
                text = unicode(recognized)
                break
        except:
            text = ""
    resp = SpeechRecoResponse()
    resp.str = text
    return resp

def asr_server():
    rospy.init_node("asr_server")
    s = rospy.Service('asr', SpeechReco, speech_reco)
    print("ASR service ready")
    asr_publisher()
    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

def asr_publisher():
    pub = rospy.Publisher('/asr_output', asr_output, queue_size=10)
    print("Publisher ready \nRunning speech reco")
    rec = sr.Recognizer()
    while True:
        try:
            with sr.Microphone() as source:
                audio = rec.listen(source, phrase_time_limit=3)
                recognized = rec.recognize_google(audio, language="en-US") #@TODO fill language from some config
                print("Heard: {}".format(recognized))
                text = str(recognized)
        except:
            text = ""
            msg = asr_output()
            msg.header.stamp = rospy.Time.now()
            msg.data = text
            pub.publish(msg)


if __name__ == "__main__":
    asr_server()


