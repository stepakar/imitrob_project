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

import owlready2 as ow

from nlp_crow.structures.tagging.TaggedText import TaggedText
from nlp_crow.utils import nlp

db = Database()

#with db.onto as onto:
class DefineAreaAction(db.onto.Template):
    namespace = db.onto
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.register_parameter(name="area_name", value=str)
        self.register_parameter(name="corner_0", value=db.onto.Location)
        self.register_parameter(name="corner_1", value=db.onto.Location)
        self.register_parameter(name="corner_2", value=db.onto.Location)
        self.register_parameter(name="corner_3", value=db.onto.Location)

    def match(self, tagged_text : TaggedText) -> None:
        # match text until "with" or "corners" is encountered
        match = tagged_text.match_regex(r"called (.*?)(?= with| corners)")

        if match:
            self.area_name = match.group(1)
        else:
            uim = UserInputManager()
            self.area_name = uim.ask_for_name("area")

        # match eight digits for area corners
        regex_pos_list = [("corners", "NN")] + ([(r".*", "CD")] * 8)
        matches = tagged_text.match_regex_pos_list(regex_pos_list)

        for i in range(1, len(matches), 2):
            coord_x = nlp.get_int(matches[i].string)
            coord_y = nlp.get_int(matches[i+1].string)
            corner = db.onto.Location(x=coord_x, y=coord_y, z=0)

            setattr(self, f"corner_{i // 2}", corner)


    def evaluate(self) -> None:
        is_default = False
        uim = UserInputManager()
        uim.say(f"Do you want to make {self.area_name} your default area? (y/n)")

        if uim.confirm_user():
            is_default = True

        self.db_api = DatabaseAPI()
        self.db_api.add_area(area_name=self.area_name,
                        corners=(self.corner_0, self.corner_1, self.corner_2, self.corner_3),
                        is_default=is_default)

        self.db_api.set_state(State.DEFINE_AREA)