#!/usr/bin/env python
"""
Copyright (c) 2019 Zdenek Kasner
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
import io
import os

import logging

# Imports the Google Cloud client library
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types

os.environ["GOOGLE_APPLICATION_CREDENTIALS"]=os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                                          "google_credentials.json")

class SpeechRecognizer:
    def __init__(self, language_code):
        self.language_code = language_code

    def transcribe(self, filename):
        # Instantiates a client
        try:
            client = speech.SpeechClient()
        except:
            err = "Could not instantiate Google Speech API. Check the path to credentials.",
            raise ConnectionRefusedError(err)

        # Loads the audio into memory
        with io.open(filename, 'rb') as audio_file:
            content = audio_file.read()
            audio = types.RecognitionAudio(content=content)

        config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
            language_code=self.language_code)

        # Detects speech in the audio file
        response = client.recognize(config, audio)
        transcript = " ".join([result.alternatives[0].transcript for result in response.results])

        return transcript
