#!/usr/bin/env python
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
from abc import abstractmethod
from typing import List, Dict, Any, ClassVar

import types

from nlp_crow.database.Database import Database
import owlready2 as ow

from nlp_crow.structures.tagging.ParsedText import ParsedText
from nlp_crow.structures.tagging.TaggedText import TaggedText

db = Database()

# TODO put everything that can be encoded in the ontology directly into the ontology
# TODO find a way to load the module automatically with the ontology


# Contains additional Python methods for classes which already exist in the OWL file.

# with db.onto:
class Template(db.onto.ComputationalThing):
    """
    A superclass for all the basic actions defined on the robot.
    """
    namespace = db.onto
    def __str__(self):
        return "{}({})".format(
            self.__class__,
            ", ".join(["{}={}".format(key, value) for key, value in self.get_inputs().items()])
        )

    @abstractmethod
    def match(self, tagged_text: TaggedText) -> None:
        """
        Extracts the value of the template parameters based on the given tagged text.
        If some parameters cannot be filled, they should be set to `None`.

        Parameters
        ----------
        tagged_text  the NL instructions as tagged text
        """
        pass

    @abstractmethod
    def evaluate(self) -> None:
        """
        Grounds the value of the parameters which are not grounded yet, e.g. ObjectPlaceholder -> Object.
        """
        pass

    def get_parameters(self) -> List[str]:
        """
        Returns a list of names of parameters which should be filled in the template.
        """
        return self.parameters

    def get_inputs(self) -> Dict[str, ow.Property]:
        """
        Returns a dictionary with template parameters and their values.
        """
        return {param : getattr(self, param) for param in self.parameters}

    def is_filled(self) -> bool:
        """
        Returns `True` if all template parameters have a value set, `False` otherwise.
        """
        return all(self.get_inputs().values())

    def register_parameter(self, name : str, value : ClassVar) -> None:
        """
        Registers a parameter which has to be filled in the template.

        Parameters
        ----------
        name Name of the parameter, will be accessed as `task.name_of_the_parameter`
        value Ontology class which will be assigned to the parameter as an input
        """
        # add the parameter name to the list of parameters
        self.parameters.append(name)

        # create dynamically a ontological property class reflecting the parameter
        types.new_class(name, (self.__class__ >> value, ow.FunctionalProperty))


class RobotProgramOperand(db.onto.RobotProgramNode):
    """
    A single instruction for the robot, an operand in the program tree.
    """
    namespace = db.onto
    def __repr__(self):
        return str(self.template) if self.template else "None"


class RobotProgramOperator(db.onto.RobotProgramNode):
    """
    An inner node in the program tree.
    """
    namespace = db.onto
    def __str__(self):
        return self.operator_type

    def add_child(self, child: db.onto.RobotProgramNode):
        self.children.append(child)


class RobotProgram(db.onto.ComputationalThing):
    """
    A program for the robot. The root of the tree (which is a RobotProgramNode) is saved in the `root` attribute.
    """
    namespace = db.onto
    class Graphics:
        """
        ASCII art for the command line.
        """
        namespace = db.onto
        dash = '\u2500'
        line = '\u2502'
        left = '\u251c\u2500\u2500 '
        right = '\u2514\u2500\u2500 '

    def __printNode(self, node: db.onto.RobotProgramNode, leftChild, levelsOpened):
        """
        Internal __str__() function helper.
        """
        string = ''.join([RobotProgram.Graphics.line + "\t" if lo else "\t" for lo in levelsOpened[:-1]])

        if node:
            if leftChild:
                string += f"{RobotProgram.Graphics.left}{node}\n"
            else:
                string += f"{RobotProgram.Graphics.right}{node}\n"

        if type(node) is RobotProgramOperator:
            for i in range(0, len(node.children)-1):
                string += self.__printNode(node.children[i], True, levelsOpened + [True])

            string += self.__printNode(node.children[-1], False, levelsOpened + [False])

        return string

    def __str__(self):
        levelsOpened = [False]

        node = self.root
        string = str(node) + '\n'

        if type(node) is RobotProgramOperator:
            for i in range(0, len(node.children)-1):
                string += self.__printNode(node.children[i], True, levelsOpened + [True])

            string += self.__printNode(node.children[-1], False, levelsOpened + [False])


        return string

class RobotCustomProgram(db.onto.RobotProgram):
    namespace = db.onto
    pass

class RobotDemonstrationProgram(db.onto.RobotCustomProgram):
    namespace = db.onto
    pass

class Area(db.onto.GeometricObject, db.onto.SpatialQuantity):
    namespace = db.onto
    def get_center(self) -> db.onto.Location:
        x = 0
        y = 0
        z = 0

        for corner in self.corners:
            x += corner.x
            y += corner.y
            z += corner.z

        # assuming a rectangular area on a grid
        return db.onto.Location(x=x//4,y=y//4,z=z//4)


class IsDefault(db.onto.Area >> bool, ow.FunctionalProperty, ow.DataProperty):
    namespace = db.onto
    python_name = "default"

class Location(db.onto.GeometricObject, db.onto.TemporalProperty):
    namespace = db.onto
    def __repr__(self):
        return f"Location(x={self.x},y={self.y},z={self.z})"

class RelativeLocation(db.onto.Location):
    namespace = db.onto
    def __str__(self):
        return f"RelativeLocation(type={self.loc_type}, relative_to={self.relative_to})"

class Object(ow.Thing):
    namespace = db.onto
    def __repr__(self):
        if self._is_in_workspace:
            str = f"{self.__class__}(name={self.name}, x={self.location.x}, y={self.location.y}, z={self.location.z}"

            if self.id:
                str += f", id={self.id}"

            if hasattr(self, "aruco_id"):
                str += f", aruco_id={self.aruco_id}"

            if self.color:
                str += f", color={self.color}"

            str += ")"
            return str
        else:
            return f"{self.__class__}"

# TODO save in the OWL file and delete from here
class Placeholder(db.onto.ComputationalThing):
    namespace = db.onto

class ObjectPlaceholder(db.onto.Placeholder):
    namespace = db.onto
    def __str__(self):
        if len(self.is_a) > 1:
            str = f"ObjectPlaceholder(cls={self.is_a[-1]}"

            if hasattr(self, "flags") and self.flags:
                str += f", flags={self.flags}"

            if hasattr(self, "color") and self.color:
                str += f", color={self.color}"

            if hasattr(self, "aruco_id"):
                str += f", aruco_id={self.aruco_id}"

            if hasattr(self, "id"):
                str += f", id={self.id}"

            str += ")"
            return str

        return f"ObjectPlaceholder(flags={self.flags})"


# TODO save in the OWL file and delete from here
class hasFlags(ObjectPlaceholder >> str, ow.DataProperty):
    namespace = db.onto
    python_name = "flags"
