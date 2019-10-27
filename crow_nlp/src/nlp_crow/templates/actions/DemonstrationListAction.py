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

#with db.onto as onto:
class DemonstrationListAction(Template):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def match(self, tagged_text : TaggedText) -> None:
        pass

    def evaluate(self) -> None:
        db_api = DatabaseAPI()
        demonstration_templates = db_api.get_demonstration_templates()

        uim = UserInputManager()
        uim.say("Actions learned from demonstration:")
        uim.show([t.name for t in demonstration_templates])