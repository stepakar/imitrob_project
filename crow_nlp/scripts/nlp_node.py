#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Karla Stepanova, Zdenek Kasner, Jan Behrens
@mail: karla.stepanova@cvut.cz
"""
import os

from database_com.msg import DBOperation
from database_com.srv import AddObjectResponse, AddObjectRequest, AddObject
from database_com.srv import SendDatabase, SendDatabaseRequest, SendDatabaseResponse
from database_com.srv._GetDatabase import GetDatabaseRequest, GetDatabase
from pynput import keyboard
import rospy
from crow_nlp.msg import SentenceProgram

# necessary to load the ontpoology classes even if seems unused
from nlp_crow.database.Ontology import *
from nlp_crow.database.Database import Database
from nlp_crow.database.DatabaseAPI import DatabaseAPI
from nlp_crow.database.Database import State
from nlp_crow.processing.NLProcessor import NLProcessor
from pprint import pprint as pp

from nlp_crow.processing.ProgramRunner import ProgramRunner
from nlp_crow.speech_recognition.speech_to_text import SpeechRecognizer
from nlp_crow.utils.files import get_full_path

import nlp_crow.utils.constants as c
import nlp_crow.utils.files as files

import logging.config


class process_sentence_node():
    # listens to /averaged_markers from object_detection package and in parallel to language input
    # (from microphone or from keyboard). For each received sentence, a program template and grounded program is created
    # via communication with database
    def __init__(self):
        rospy.init_node('process_sentence_node', anonymous=False)

        logging.config.fileConfig(get_full_path('config', 'logging.ini'))

        self._getdatsrv = rospy.ServiceProxy('/db_interface_node/getDatabase', GetDatabase)

        req = GetDatabaseRequest()
        req.write = False
        res = self._getdatsrv.call(req)


        self.sub = rospy.Subscriber('/nl_input', SentenceProgram, queue_size=10, callback=self.callback)
        self.pub = rospy.Publisher('/sentence_output', SentenceProgram, queue_size=10)
        self._addsrv = rospy.ServiceProxy('/db_interface_node/addObject', AddObject)
        self._senddatsrv = rospy.ServiceProxy('/db_interface_node/sendDatabase', SendDatabase)

        self.db_api = DatabaseAPI()
        self.db = self.db_api.get_db()
        self.ontoC = self.db.change_onto(res.path)

        #obj = self.db.onto.search(type = db.onto.Cube)
        #obj2 = self.ontoC.search(type = db.onto.Cube)

        #obj3 = self.db.onto.search(type = db.onto.Cube)
        #obj4 = self.ontoC.search(type = db.onto.Cube)

        #path = '/home/algernon/ros_melodic_ws/base_ws/src/crow_nlp/scripts/saved_updated_onto.owl'


        self.UngroundedID = 1
        self.GroundedID = 1


    def callback(self, data):
        input_sentences = data.data
        self.process_sentences(input_sentences)
        return

    def process_sentences(self, input_sentences):
        # populate the workspace
        #TODO should be replaced by input from a realsense
        self.add_object('Glue', x=0.2, y=0.4, z=0, color='black')
        self.add_object('Panel', x=0.2, y=0.1, z=0, color = 'red')
        self.add_object('Panel', x=0.2, y=-0.2, z=0, color = 'blue')

        self.add_object('Cube', x=0.1, y=0.3, z=0, id='0', color='red')
        self.add_object('Cube', x=0.1, y=0.2, z=0, id='1', color='red')
        self.add_object('Cube', x=0.2, y=0.2, z=0, id='2', color='green')

        # get current database after adding all objects

        self.get_database(write=False)

        # just to see if objects were added to the database
        obj = self.db.onto.search(type=db.onto.Cube)
        print(obj)
        obj = self.db.onto.search(type=db.onto.Glue)
        print(obj)
        obj = self.db.onto.search(type=db.onto.Panel)
        print(obj)

        for input_sentence in input_sentences:

            self.get_database(write=True)

            #self.ontoC = self.db.onto.__enter__()
            nl_processor = NLProcessor()
            program_template = nl_processor.process_text(input_sentence)
            # get current database state for writing an ungrounded and currently grounded program to be executed

            print()
            print("Program Template")
            print("--------")
            print(program_template)

            if self.db_api.get_state() == State.DEFAULT:
                # self.save_unground_program(program_template)
                self.send_database()
                self.get_database(write=True)

               # self.ontoC = self.db.onto.__enter__()
                robot_program = self.run_program(program_template)
                if self.db_api.get_state() == State.DEFAULT:
                    self.save_grounded_program(robot_program)
                    self.send_database()
                elif self.db_api.get_state() != State.LEARN_FROM_INSTRUCTIONS:
                    self.db_api.set_state(State.DEFAULT)
                    self.send_database()

            elif self.db_api.get_state() == State.LEARN_FROM_INSTRUCTIONS:
                self.save_new_template(program_template)
                self.send_database()

        # print list of programs
        self.get_database(write=False)
        all_custom_templates = self.db_api.get_custom_templates()
        for custom_template in all_custom_templates:
            print(custom_template.name[1:])
        all_programs = self.db.onto.search(type=self.db.onto.RobotProgram)
        path = os.path.dirname(os.path.abspath(__file__)) + '/saved_updated_onto.owl'
        for program in all_programs:
            print(program.name)
        return

    def act(self, program_template):
        state = self.db_api.get_state()

        if state == State.DEFAULT:
            self.run_program(program_template)

        elif state == State.LEARN_FROM_INSTRUCTIONS:
            self.db_api.add_custom_template(program_template)
            self.db_api.set_state(State.DEFAULT)

    def run_program(self, program_template):
        program_runner = ProgramRunner()
        robot_program = program_runner.evaluate(program_template)
        print()
        print("Grounded Program")
        print("--------")
        print(robot_program)
        return robot_program

    def save_new_template(self, program_template):
        self.db_api.add_custom_template(program_template)
        self.db_api.set_state(State.DEFAULT)
        return

    def save_unground_program(self, program_template):
        # save to database and when database with the program sent,
        # we sent a message that the ground program was written to
        # database and new database sent
        # TODO add parameter time to the added program
        # TODO add parameter to be done to the added program
        # TODO search ontology for last program id
        name = 'ungrounded_' + str(self.UngroundedID)
        self.UngroundedID = self.UngroundedID + 1
        self.db_api.save_program(program_template, name)
        return

    def save_grounded_program(self, ground_program):
        # save to database and when database with the program sent,
        # we sent a message that the ground program was written to
        # database and new database sent
        # TODO search ontology for last program id
        # TODO link the corresponding ungrounded program in grounded one or vice versa
        # TODO add parameter time to the added program
        name = 'grounded_' + str(self.GroundedID)
        self.GroundedID = self.GroundedID + 1
        self.db_api.save_program(ground_program, name)

        return

    def get_database(self, write):
        req = GetDatabaseRequest()
        req.write = write
        res = self._getdatsrv.call(req)

        self.ontoC = self.db.change_onto(res.path)
        obj = self.ontoC.search(type=self.ontoC.Cube)
        if write:
            self.ontoC = self.db.onto.__enter__()
        return

    def send_database(self):
        req = SendDatabaseRequest()
        self.ontoC.__exit__()
        path = os.path.dirname(os.path.abspath(__file__)) + '/saved_updated_onto.owl'
        self.ontoC.save(path)
        # rospy.sleep(1.0)

        req.path = path
        res = self._senddatsrv.call(req)
        print(res.received.msg)
        print(res.received.success)
        # self.db.onto.__exit__()
        return

    def add_object(self, type, x, y, z, id=None, color=None):
        req = AddObjectRequest()
        req.obj_class = type
        req.x = x
        req.y = y
        req.z = z
        if id:
            req.id = id
        if color:
            req.color = color
        req.action.action = DBOperation.ADD
        res = self._addsrv.call(req)
        assert isinstance(res, AddObjectResponse)
        return


if __name__ == '__main__':
    process_sentence_node()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()