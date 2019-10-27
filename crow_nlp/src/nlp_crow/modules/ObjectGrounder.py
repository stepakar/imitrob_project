#!/usr/bin/env python
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
from enum import Enum
from typing import ClassVar, Any

from nlp_crow.database.Database import Database
from nlp_crow.database.DatabaseAPI import DatabaseAPI
from nlp_crow.modules.UserInputManager import UserInputManager
from nlp_crow.modules.ColorDetector import ColorDetector

import logging
import owlready2 as ow

db = Database()

# with db.onto as onto:
class ObjectGrounder:
    namespace = db.onto
    #class Flags(Enum):
    #     CAN_BE_PICKED = 1

    def __init__(self):
        self.db_api = DatabaseAPI()
        self.cd = ColorDetector()
        self.ar = UserInputManager()

        self.logger = logging.getLogger(__name__)

    def ground_object(self, obj_placeholder, flags=()) -> Any:
        # try to find out the class of the object
        # the 0th item of an is_a list should be always ObjectPlaceholder
        # if the item is bound to a real class, the class will be the 1st item
        cls = obj_placeholder.is_a[-1]

        if "last_mentioned" in obj_placeholder.flags:
            objs = [self.db_api.get_last_mentioned_object()]

        else:
            props = obj_placeholder.get_properties()
            props.discard(db.onto.hasFlags) # this is internal property of the object placeholder

            # put together a dictionary of properties required from the object
            props_vals = {self.get_prop_name(prop): getattr(obj_placeholder, self.get_prop_name(prop)) for prop in props}

            # find if there is an object with such properties in the workspace
            objs = self.db_api.get_by_properties(cls=cls, properties=props_vals)

        # if self.Flags.CAN_BE_PICKED in flags:
        #     objs = list(filter(lambda x: x._can_be_picked, objs))

        # only one object found / any object can be selected
        if len(objs) == 1 or ("any" in obj_placeholder.flags and len(objs) > 0):
            obj = objs[0]
            self.db_api.set_last_mentioned_object(obj)
        # more objects -> ask the user to select one
        elif len(objs) > 1:
            obj = self.ar.ask_to_select_from_class_objects(objs)
            self.db_api.set_last_mentioned_object(obj)
        else:
            self.logger.warning(f"No object of type {cls} in the workspace.")
            obj = None

        if obj:
            self.logger.debug(f"Object found for {obj_placeholder}: {obj}")

        return obj


    def get_prop_name(self, prop : ow.DataPropertyClass):
        # we need to use the python name of properties whenever it is defined
        if hasattr(prop, "python_name"):
            return prop.python_name

        return prop.name