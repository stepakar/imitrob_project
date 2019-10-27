#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Karla Stepanova
@mail: karla.stepanova@cvut.cz
"""
import os

import rospy
# from database_com.srv import GroundProgram, GroundProgramResponse, GroundProgramRequest
from database_com.srv import AddObject, AddObjectResponse, AddObjectRequest
from database_com.srv import GetDatabase, GetDatabaseResponse, GetDatabaseRequest
from database_com.srv import SendDatabase, SendDatabaseResponse, SendDatabaseRequest

from geometry_msgs.msg import PoseStamped
from object_detection_aruco.msg import MarkersAvgList, MarkersAvg
from database_com.msg import DatabaseUpdate
import logging
from nlp_crow.utils.files import get_full_path


# necessary to load the ontology classes even if seems unused
from nlp_crow.processing.ProgramExecutor import ProgramExecutor
from nlp_crow.database.Ontology import *
from nlp_crow.database.Database import Database
from nlp_crow.database.DatabaseAPI import DatabaseAPI
from pynput import keyboard

import owlready2 as ow
import sys

from nlp_crow.processing.ProgramRunner import ProgramRunner

import owlready2 as ow
import logging.config


class execute_program_node():
    def __init__(self):
        rospy.init_node('execute_program_node', anonymous=False)

        logging.config.fileConfig(get_full_path('config', 'logging.ini'))

        self._getdatsrv = rospy.ServiceProxy('/db_interface_node/getDatabase', GetDatabase)

        req = GetDatabaseRequest()
        req.write = False
        res = self._getdatsrv.call(req)

        # self.db = Database(res.path)
        self._addsrv = rospy.ServiceProxy('/db_interface_node/addObject', AddObject)
        self._senddatsrv = rospy.ServiceProxy('/db_interface_node/sendDatabase', SendDatabase)

        self.db_api = DatabaseAPI()
        # self.onto = self.db_api.get_onto()
        # path = '/home/algernon/ros_melodic_ws/base_ws/src/crow/database_com/nodes/saved_onto.owl'
        self.db = self.db_api.get_db()
        self.db.change_onto(res.path)
        obj = self.db.onto.search(type=self.db.onto.Cube)

        # exit_res = self.db.onto.__exit__()

        print('press a key to execute an action')
        self.robotProgram = 'grounded_1'
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

      #  rospy.on_shutdown(self.shutdown_hook)

    def on_press(self,key):
        """
        Executes the robotic program
        """
        print('hi')
        self.get_database(write=False)
        all_programs = self.db.onto.search(type=self.db.onto.RobotProgram)
        program_grounded_all = [x for x in all_programs if x.name.startswith("grounded")]
        for program_grounded in program_grounded_all:
            print(program_grounded.name)
            print(program_grounded)
            program_executor = ProgramExecutor()
            program_executor.evaluate(program_grounded)


        return

    def get_database(self, write):
        req = GetDatabaseRequest()
        # self.ontoC = self.db.onto.__exit__()
        req.write = write
        res = self._getdatsrv.call(req)
        # self.db_api = DatabaseAPI()

        # self.db = self.db_api.get_db()
        self.db.change_onto(res.path)
        #path = '/home/algernon/ros_melodic_ws/base_ws/src/crow_nlp/scripts/saved_updated_onto.owl'
        #self.ontoC = self.db.change_onto(path)
        self.db.onto.__enter__()
        obj = self.db.onto.search(type=self.db.onto.Cube)
        self.db.onto.__exit__()
        # self.ontoC = self.db.onto.__enter__()
        print('hi')
        return

if __name__ == '__main__':
    execute_program_node()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()