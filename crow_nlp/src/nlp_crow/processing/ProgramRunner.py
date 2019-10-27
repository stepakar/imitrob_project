#!/usr/bin/env python
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
from nlp_crow.database.Database import Database
from nlp_crow.database.DatabaseAPI import DatabaseAPI
from nlp_crow.modules.LocationGrounder import LocationGrounder
from nlp_crow.modules.ObjectGrounder import ObjectGrounder

import owlready2 as ow
import logging

from nlp_crow.templates.TemplateFactory import TemplateType

db = Database()

# with db.onto as onto:
class ProgramRunner:
    """
    Grounds object and location placeholders in the template so that they refer
    to real objects and locations in the workspace. The result can be used to
    create instructions for the robot.
    """
    def __init__(self):
        self.logger = logging.getLogger(__name__)

        self.og = ObjectGrounder()
        self.lg = LocationGrounder()
        self.db_api = DatabaseAPI()

    def evaluate(self, program : db.onto.RobotProgram):
        self.evaluate_recursive(program.root)

        return program

    def evaluate_recursive(self, node : db.onto.RobotProgramNode) -> None:
        if type(node) == db.onto.RobotProgramOperator:
            self.evaluate_operator(node)

        elif type(node) == db.onto.RobotProgramOperand:
            self.evaluate_operand(node)


    def evaluate_operator(self, node: db.onto.RobotProgramOperator):
        for child in node.children:
            self.evaluate_recursive(child)

    def evaluate_operand(self, node: db.onto.RobotProgramOperand):
        template = node.template

        if template is None:
            return

        # call the evaluate() method for each template
        template.evaluate()

        for param, input in template.get_inputs().items():
            if not input:
                self.logger.error(f"Could not fill {param}")