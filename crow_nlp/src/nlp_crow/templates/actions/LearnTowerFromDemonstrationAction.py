#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""

import itertools
import logging
import time
from typing import List, Dict

from nlp_crow.database.DatabaseAPI import DatabaseAPI
from nlp_crow.modules.DemonstrationLearner import DemonstrationLearner
from nlp_crow.modules.GrammarParser import GrammarParser
from nlp_crow.modules.UserInputManager import UserInputManager


from nlp_crow.database.Database import Database, State
from nlp_crow.database.Ontology import Template, RobotProgramOperand, ObjectPlaceholder, RelativeLocation, Location

import owlready2 as ow

from nlp_crow.structures.tagging.TaggedText import TaggedText
from nlp_crow.templates.tasks.PickTask import PickTask
from nlp_crow.templates.tasks.PutTask import PutTask

db = Database()

#with db.onto as onto:
class LearnTowerFromDemonstrationAction(Template):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.logger = logging.getLogger(__name__)

        self.register_parameter(name="task_name", value=str)


    def match(self, tagged_text : TaggedText) -> None:
        match = tagged_text.match_regex(r"called (.*)")

        if match:
            self.task_name = match.group(1)
        else:
            uim = UserInputManager()
            self.task_name = uim.ask_for_name("tower")


    def evaluate(self) -> None:
        self.db_api = DatabaseAPI()
        self.db_api.set_param("current_task_name", self.task_name)

        dl = DemonstrationLearner()

        program = dl.create_new_program()
        last_cube_positions = {}

        for cube_positions in dl.get_cube_positions():
            for key, val in cube_positions.items():
                val
                self.add_object('Cube', x=0.1, y=0.3, z=0, id=key, color='red')
            logging.debug(f"Position of cubes: {cube_positions}")

            overlapping = self.detect_overlapping(last_cube_positions, cube_positions)

            if overlapping:
                bottom_cube_id = overlapping[0]
                top_cube_id = overlapping[1]

                logging.debug(f"Overlapping cubes detected: cube id {top_cube_id} is on top of cube id {bottom_cube_id}")

                if not program.root.children:
                    # beginning of the demonstration, place the first cube
                    self.add_pick_task(program, bottom_cube_id)
                    self.add_put_first_cube_task(program, bottom_cube_id)

                self.add_pick_task(program, top_cube_id)
                self.add_put_task(program, top_cube_id, bottom_cube_id)

            last_cube_positions = cube_positions
            # time.sleep(1)

        logging.debug("Demonstration finished, adding program to database")
        logging.debug("\n" + str(program))

        self.db_api.add_custom_template(program)
        self.db_api.set_state(State.LEARN_FROM_DEMONSTRATION)



    def detect_overlapping(self, last_cube_positions : Dict, cube_positions : Dict):
        cubes_disappeared = list(set(last_cube_positions.keys()).difference(set(cube_positions.keys())))

        if cubes_disappeared:
            cube_disappeared_id = cubes_disappeared[0]  # caring only about the first cube now

            cube_disappeared_pos = last_cube_positions[cube_disappeared_id]

            for cube_id, cube_pos in cube_positions.items():
                if cube_pos.x == cube_disappeared_pos.x and cube_pos.y == cube_disappeared_pos.y:
                    return [cube_disappeared_id, cube_id]

        return []


    def add_pick_task(self, program, top_cube_id):
        pick_task = PickTask()
        top_cube = db.onto.ObjectPlaceholder()
        top_cube.is_a.append(db.onto.Cube)
        top_cube.id = top_cube_id

        pick_task.object_to_pick = top_cube
        self.add_task(program, pick_task)


    def add_put_task(self, program, top_cube_id, bottom_cube_id):
        put_task = PutTask()

        top_cube = db.onto.ObjectPlaceholder()
        top_cube.is_a.append(db.onto.Cube)
        top_cube.id = top_cube_id

        put_task.object_to_put = top_cube

        bottom_cube = db.onto.ObjectPlaceholder()
        bottom_cube.is_a.append(db.onto.Cube)
        bottom_cube.id = bottom_cube_id

        put_task.location = RelativeLocation()
        put_task.location.loc_type = "top"
        put_task.location.relative_to = bottom_cube

        self.add_task(program, put_task)


    def add_put_first_cube_task(self, program, cube_id):
        put_task = PutTask()

        top_cube = db.onto.ObjectPlaceholder()
        top_cube.is_a.append(db.onto.Cube)
        top_cube.id = cube_id

        put_task.object_to_put = top_cube

        db_api = DatabaseAPI()
        default_area = db_api.get_default_area()

        put_task.location = RelativeLocation()
        put_task.location.loc_type = "center"
        put_task.location.relative_to = default_area

        self.add_task(program, put_task)


    def add_task(self, program, template):
        node = RobotProgramOperand()
        node.template = template

        program.root.add_child(node)