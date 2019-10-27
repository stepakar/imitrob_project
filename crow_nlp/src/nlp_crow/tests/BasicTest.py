#!/usr/bin/env python
"""
Copyright (c) 2019 Zdenek Kasner
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
import unittest
from pprint import pprint as pp
from nlp_crow.database.DatabaseAPI import DatabaseAPI
from nlp_crow.processing.NLProcessor import NLProcessor


class BasicTest(unittest.TestCase):

    def setUp(self):
        self.db = DatabaseAPI()
        self.onto = self.db.get_onto()


    def test_add_and_remove_tools(self):
        with self.onto as onto:
            # populate the workspace
            self.db.add_object(onto.Screwdriver, x=1, y=2, color="red")
            self.db.add_object(onto.Screwdriver, x=3, y=1, color="blue")
            self.db.add_object(onto.Hammer, x=4, y=4, color="blue")
            self.db.add_object(onto.Pliers, x=3, y=3, color="green")

            # try a few queries about the workspace
            print("All objects in the workspace:")
            pp(self.db.get_all_tools())
            print()

            print("All screwdrivers in the workspace:")
            pp(self.db.get_class_objects(onto.Screwdriver))
            print()

            print("All blue objects in the workspace:")
            pp(self.db.get_objects_by_color("blue"))
            print()

            # # update position of the pliers
            # pliers = self.db.get_class_objects(onto.Pliers)[0]
            # self.db.update_object_position(pliers, x=5, y=4)
            # print("The pliers after updating its position:")
            # print(pliers)
            # print()

            # delete the nail
            print("Deleting the pliers...")
            self.db.delete_object(pliers)
            print("List of pliers:")
            pp(self.db.get_class_objects(onto.Pliers))
            print()

            # process a NL instruction
            nl_processor = NLProcessor()

            input_sentence = "Now take the screwdriver and put it in the right corner."
            robot_program = nl_processor.process_text(input_sentence)

            print()
            print("Program")
            print("--------")
            print(robot_program)



if __name__ == '__main__':
    unittest.main()