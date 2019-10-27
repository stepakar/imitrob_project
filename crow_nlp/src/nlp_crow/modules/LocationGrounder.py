#!/usr/bin/env python
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
from typing import ClassVar, Any

from nlp_crow.database.Database import Database
from nlp_crow.database.DatabaseAPI import DatabaseAPI
from nlp_crow.modules.ObjectGrounder import ObjectGrounder
from nlp_crow.modules.UserInputManager import UserInputManager
from nlp_crow.modules.ColorDetector import ColorDetector

import logging
import owlready2 as ow

db = Database()

# with db.onto as onto:
class LocationGrounder:
    """
    Grounds relative locations to absolute locations based on the current state of the workspace.
    """
    namespace = db.onto
    def __init__(self):
        self.db_api = DatabaseAPI()
        self.ar = UserInputManager()
        self.og = ObjectGrounder()
        self.logger = logging.getLogger(__name__)

    def ground_location(self, loc : db.onto.RelativeLocation) -> Any:
        if type(loc.relative_to) is db.onto.Area:
            abs_loc = self.calculate_absolute_location_from_area(loc, loc.relative_to)
        else:
            # it is necessary to ground the object first
            obj = self.og.ground_object(obj_placeholder=loc.relative_to)

            if not obj:
                return None

            abs_loc = self.calculate_absolute_location_from_object(loc, obj)
        return abs_loc

    def calculate_absolute_location_from_area(self, loc_template, area):
        type = loc_template.loc_type
        loc = area.get_center()

        if type == "center":
            pass

        elif type == "right":
            loc.x += 1

        elif type == "left":
            loc.x -= 1

        if loc:
            self.logger.debug(f"Location found for {loc_template}: {loc}")

        return loc

    def calculate_absolute_location_from_object(self, loc_template, obj):
        type = loc_template.loc_type
        loc = db.onto.Location()

        x = obj.location.x
        y = obj.location.y
        z = obj.location.z

        loc.x = x
        loc.y = y
        loc.z = z

        if type == "center":
            pass

        elif type == "right":
            loc.x = x + 0.1

        elif type == "left":
            loc.x = x - 0.1

        elif type == "top":
            loc.z = z + 0.1

        elif type == "bottom":
            loc.z = z - 0.1

        # TODO etc.
        if loc:
            self.logger.debug(f"Location found for {loc_template}: {loc}")


        return loc
