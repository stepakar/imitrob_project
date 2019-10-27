#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""

from nlp_crow.database.Database import Database
from nlp_crow.database.DatabaseAPI import DatabaseAPI
from nlp_crow.database.Ontology import Template
from nlp_crow.modules.ObjectDetector import ObjectDetector

import owlready2 as ow

from nlp_crow.modules.ObjectGrounder import ObjectGrounder
from nlp_crow.structures.tagging.TaggedText import TaggedText

import logging

db = Database()

# with db.onto as onto:
class PickTask(Template):
    """
    A template for the pick task = a robot instruction representing picking a specific object.
    """
    namespace = db.onto
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.register_parameter(name="object_to_pick", value=db.onto.Object)
        self.logger = logging.getLogger(__name__)
        self.db_api = DatabaseAPI()

    def match(self, tagged_text : TaggedText) -> None:
        od = ObjectDetector()

        self.object_to_pick = od.detect_object(tagged_text)

        # if self.object_to_pick:
            # this is not changing the object state, but setting a necessary condition for the object to be picked
            # self.object_to_pick._can_be_picked = True

    def evaluate(self) -> None:
        # picked_object = self.db_api.get_picked_object()
        #
        # if picked_object is not None:
        #     self.logger.warning(f"Cannot pick an object, robot is already holding {picked_object}.")
        #     self.object_to_pick = None
        #     return

        og = ObjectGrounder()
        self.object_to_pick = og.ground_object(obj_placeholder=self.object_to_pick)
        print(self)
        # self.object_to_pick = og.ground_object(obj_placeholder=self.object_to_pick, flags=[ObjectGrounder.Flags.CAN_BE_PICKED])

        # if self.object_to_pick:
        #     # this is changing the object state
        #     self.object_to_pick._is_picked = True
        #     self.object_to_pick._can_be_picked = False


    def execute(self) -> None:
        self.object_to_pick.location.x
        self.object_to_pick.location.y
        print(self)

        