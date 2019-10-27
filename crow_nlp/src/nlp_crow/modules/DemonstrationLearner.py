#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""

from nlp_crow.database.Database import Database
from nlp_crow.database.Ontology import RobotProgram, RobotProgramOperator, RobotDemonstrationProgram
import random


db = Database()

#with db.onto as onto:
class DemonstrationLearner():
    namespace = db.onto
    def create_new_program(self):
        program = RobotDemonstrationProgram()

        # hardcoded program structure: all subprograms are located directly under the root node "AND"
        program.root = RobotProgramOperator(operator_type="AND")

        return program


    def get_cube_positions(self):
        l = [
            {"0" : db.onto.Location(x=2,y=2,z=0), "1" :  db.onto.Location(x=1,y=1,z=0), "2" : db.onto.Location(x=3,y=1,z=0)},
            {"0" : db.onto.Location(x=2,y=2,z=0), "1" :  db.onto.Location(x=1,y=1,z=0), "2" : db.onto.Location(x=3,y=1,z=0)},
            {"0" : db.onto.Location(x=2,y=2,z=0), "1" :  db.onto.Location(x=1,y=1,z=0), "2" : db.onto.Location(x=3,y=1,z=0)},
            {"0" : db.onto.Location(x=2,y=2,z=0), "1" :  db.onto.Location(x=1,y=1,z=0), "2" : db.onto.Location(x=2,y=1,z=0)},
            {"1" :  db.onto.Location(x=1,y=1,z=0), "2" : db.onto.Location(x=2,y=2,z=1)},
            {"1" :  db.onto.Location(x=1,y=1,z=0), "2" : db.onto.Location(x=2,y=2,z=1)},
            {"1" :  db.onto.Location(x=2,y=1,z=1), "2" : db.onto.Location(x=2,y=2,z=1)},
            {"1" :  db.onto.Location(x=2,y=2,z=2)},
            {"1" :  db.onto.Location(x=2,y=2,z=2)},
            {"1" :  db.onto.Location(x=2,y=2,z=2)},
            {"1" :  db.onto.Location(x=2,y=2,z=2)}
        ]

        for x in l:
            yield x