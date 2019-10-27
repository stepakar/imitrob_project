#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Karla Stepanova
@e-mail: karla.stepanova@cvut.cz
"""
import rospy

from nlp_crow.database.Database import Database
from nlp_crow.database.DatabaseAPI import DatabaseAPI
from nlp_crow.modules.LocationGrounder import LocationGrounder
from nlp_crow.modules.ObjectGrounder import ObjectGrounder

from capek_motion_skills.service_proxies import KukaServiceProxies
import owlready2 as ow
import logging

from nlp_crow.templates.TemplateFactory import TemplateType

db = Database()

# with db.onto as onto:
class ProgramExecutor:

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

        #template.evaluate()
        z_ref = 0
        #TODO if template.is_a APPLY GLUE TASK/PICK TASK
        if isinstance(template, db.onto.PickTask):
            x_disc = template.object_to_pick.location.x
            y_disc = template.object_to_pick.location.y

            KukaServiceProxies.pick_approach('r2_arm', x=x_disc, y=y_disc, z=z_ref + 0.1, frame_id='table2', ee_offset=0.117)
            KukaServiceProxies.pick_approach('r2_arm', x=x_disc, y=y_disc, z=z_ref, frame_id='table2', ee_offset=0.117)
            rospy.sleep(0.5)
            #KukaServiceProxies.magnet_gripper('r2_arm', on=True)
            rospy.sleep(0.5)
            KukaServiceProxies.pick_approach('r2_arm', x=x_disc, y=y_disc, z=z_ref + 0.1, frame_id='table2', ee_offset=0.117)
            rospy.sleep(1)
        elif isinstance(template, db.onto.PutTask):
            print('template is a PutTask, which is yet to be implemented: {}'.format(template))
            x_disc = template.object_to_pick.location.x
            y_disc = template.object_to_pick.location.y

            KukaServiceProxies.pick_approach('r2_arm', x=x_disc, y=y_disc, z=z_ref + 0.1, frame_id='table2',
                                             ee_offset=0.117)
            KukaServiceProxies.pick_approach('r2_arm', x=x_disc, y=y_disc, z=z_ref, frame_id='table2', ee_offset=0.117)
            rospy.sleep(0.5)
            # KukaServiceProxies.magnet_gripper('r2_arm', on=True)
            rospy.sleep(0.5)
            KukaServiceProxies.pick_approach('r2_arm', x=x_disc, y=y_disc, z=z_ref + 0.1, frame_id='table2',
                                             ee_offset=0.117)
            rospy.sleep(1)
        elif isinstance(template, db.onto.ApplyGlueTask):
            print('template is a ApplyGlueTask, which is now working same as PickObjectTask: {}'.format(template))
            x_disc = template.position.x
            y_disc = template.position.y

            KukaServiceProxies.pick_approach('r2_arm', x=x_disc, y=y_disc, z=z_ref + 0.1, frame_id='table2', ee_offset=0.117)
            KukaServiceProxies.pick_approach('r2_arm', x=x_disc, y=y_disc, z=z_ref, frame_id='table2', ee_offset=0.117)
            rospy.sleep(0.5)
            #KukaServiceProxies.magnet_gripper('r2_arm', on=True)
            rospy.sleep(0.5)
            KukaServiceProxies.pick_approach('r2_arm', x=x_disc, y=y_disc, z=z_ref + 0.1, frame_id='table2', ee_offset=0.117)
            rospy.sleep(1)
        else:
            print('template has no executor: {}'.format(template))
