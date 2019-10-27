#!/usr/bin/env python
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
import nlp_crow.modules.LocationDetector as LocationDetector
from nlp_crow.structures.tagging.MorphCategory import POS
from nlp_crow.structures.tagging.Tag import Tag
from nlp_crow.structures.tagging.TaggedText import TaggedText

db = Database()
# with db.onto as onto:
class ObjectDetector(CrowModule):
    """
    Detects an object in text.
    """
    namespace = db.onto
    def __init__(self):
        self.logger = logging.getLogger(__name__)

        self.db_api = DatabaseAPI()

        self.class_map = {
            "screwdriver": db.onto.Screwdriver,
            "hammer": db.onto.Hammer,
            "pliers": db.onto.Pliers,
            "glue" : db.onto.Glue,
            "panel" : db.onto.Panel,
            "cube" : db.onto.Cube
        }

    def detect_object(self, tagged_text : TaggedText) -> db.onto.Object:
        """
        Detects an object mentioned in the input text, extracts its properties and saves it
        in the object placeholder.

        Parameters
        ----------
        tagged_text  an input text

        Returns
        -------
        an object placeholder to be grounded later
        """
        obj = None
        text = tagged_text.get_text()

        # try to detect one of the known objects in text
        for obj_str in self.class_map.keys():
            if tagged_text.contains_pos_token(obj_str, "NN"):
                obj = self.detect_explicit_object(tagged_text, obj_str)
                break

        # try to detect a coreference to an object
        if obj is None and tagged_text.contains_pos_token("it", "PRP"):
            obj = self.detect_coreferenced_object()

        self.logger.debug(f"Object detected for \"{text}\": {obj}")

        return obj

    def detect_explicit_object(self, tagged_text, obj_str):
        """
        Detect an object which is mentioned explicitly.
        """
        cls = self.class_map[obj_str]
        obj = db.onto.ObjectPlaceholder()
        obj.is_a.append(cls)

        if tagged_text.contains_text("any " + obj_str):
            # the "any" flag will be used to select any object without asking the user
            obj.flags.append("any")

        self.detect_object_color(obj, tagged_text)
        self.detect_object_id(obj, tagged_text)
        self.detect_object_location(obj, obj_str, tagged_text)

        return obj

    # def detect_known_object(self, tagged_text, obj_str):
    #     cls = self.class_map[obj_str]
    #     obj = db.onto.ObjectPlaceholder()
    #     obj.is_a.append(cls)
    #
    #     if tagged_text.contains_text("any " + obj_str):
    #         obj.flags.append("any")
    #
    #     self.detect_object_color(obj, tagged_text)
    #     self.detect_object_location(obj, obj_str, tagged_text)
    #
    #     return obj

    def detect_coreferenced_object(self):
        """
        Detect that the text is referencing an object mentioned earlier.
        """

        obj = db.onto.ObjectPlaceholder()
        obj.flags.append("last_mentioned")

        return obj

    def detect_object_location(self, obj, obj_str, tagged_text):
        # cut the part of the text that is sent into the location detector to avoid infinite loop
        # TODO: all of this is only a temporary solution, not intended to be used in the final product
        end_index = tagged_text.get_text().find(obj_str) + len(obj_str)
        new_tagged_text = tagged_text.cut(end_index, None)

        ld = LocationDetector.LocationDetector()
        location = ld.detect_location(new_tagged_text)
        if location:
            obj.location = location

    def detect_object_color(self, obj, tagged_text):
        cd = ColorDetector.ColorDetector()
        color = cd.detect_color(tagged_text)

        if color:
            obj.color.append(db.onto.NamedColor(color))

    def detect_object_id(self, obj, tagged_text):
        idet = IdDetector.IdDetector()

        id = idet.detect_id(tagged_text)

        if id is not None:
            obj.aruco_id = id