#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
from nlp_crow.database.Database import Database
from nlp_crow.templates.actions.DefineAreaAction import DefineAreaAction
from nlp_crow.templates.actions.DemonstrationListAction import DemonstrationListAction
from nlp_crow.templates.actions.LearnNewTaskAction import LearnNewTaskAction
from nlp_crow.templates.actions.LearnTowerFromDemonstrationAction import LearnTowerFromDemonstrationAction
from nlp_crow.templates.tasks.ApplyGlueTask import ApplyGlueTask
from nlp_crow.templates.tasks.PickTask import PickTask

from enum import Enum

from nlp_crow.templates.tasks.PutTask import PutTask


class TemplateType(Enum):
    # mapping from constants to classes
    PICK_TASK = PickTask
    APPLY_GLUE = ApplyGlueTask
    PUT_TASK = PutTask

    LEARN_NEW_TASK = LearnNewTaskAction
    LEARN_TOWER = LearnTowerFromDemonstrationAction
    DEMONSTRATION_LIST = DemonstrationListAction
    DEFINE_AREA = DefineAreaAction


db = Database()


# with db.onto as onto:
class TemplateFactory:
    namespace = db.onto
    def get_template(self, type : TemplateType) -> db.onto.Template:
        return type.value()
