#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""

from nlp_crow.database.DatabaseAPI import DatabaseAPI
from nlp_crow.modules.UserInputManager import UserInputManager


from nlp_crow.database.Database import Database, State
from nlp_crow.database.Ontology import Template

import owlready2 as ow

from nlp_crow.structures.tagging.TaggedText import TaggedText

db = Database()

# with db.onto as onto:
class LearnNewTaskAction(Template):
    namespace = db.onto
    """
    A template for the learn-new-task action. Detects that the user wants to teach the robot a new
    compound action.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.register_parameter(name="task_name", value=str)

    def match(self, tagged_text : TaggedText) -> None:
        match = tagged_text.match_regex(r"called (.*)")

        if match:
            self.task_name = match.group(1)
        else:
            uim = UserInputManager()
            self.task_name = uim.ask_for_name("task")


    def evaluate(self) -> None:
        db_api = DatabaseAPI()

        db_api.set_param("current_task_name", self.task_name)
        db_api.set_state(State.LEARN_FROM_INSTRUCTIONS)