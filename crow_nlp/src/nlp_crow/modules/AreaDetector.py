#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""

import logging
from typing import Any

from nlp_crow.database.Database import Database
from nlp_crow.database.DatabaseAPI import DatabaseAPI
from nlp_crow.modules import IdDetector
from nlp_crow.modules.CrowModule import CrowModule
import nlp_crow.modules.ColorDetector as ColorDetector
import nlp_crow.modules.ObjectGrounder as ObjectGrounder
#import nlp_crow.modules.LocationDetector as LocationDetector
from nlp_crow.structures.tagging.MorphCategory import POS
from nlp_crow.structures.tagging.Tag import Tag
from nlp_crow.structures.tagging.TaggedText import TaggedText

db = Database()
#with db.onto as onto:
class AreaDetector(CrowModule):
    namespace = db.onto
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.db_api = DatabaseAPI()


    def detect_area(self, tagged_text : TaggedText) -> db.onto.ObjectPlaceholder:
        area_list = self.db_api.get_class_objects(db.onto.Area)

        for area in area_list:
            if tagged_text.contains_text(area.name):
                self.logger.debug(f"Area detected for \"{tagged_text.get_text()}\": {area}")

                return area

        return None