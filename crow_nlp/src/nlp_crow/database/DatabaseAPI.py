#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
import logging

# necessary to load the ontology classes even if seems unused
from typing import Tuple

from nlp_crow.database.Ontology import *
from nlp_crow.database.Database import Database, State

import owlready2 as ow

from nlp_crow.utils.config import Config

db = Database()


# with db.onto:
class DatabaseAPI:
    """
    Provides access to the ontology database.
    """
    def __init__(self):
        self.logger = logging.getLogger(__name__)

    def add_object(self, Class: ClassVar, x : float, y : float, z : float, id : str = None, color : str = None):
        """
        Adds a new object in the ontology.

        Parameters
        ----------
        Class the class of the ontology object to be added, should be a subclass of onto.Object
        x     the position of the object: x coordinate
        y     the position of the object: y coordinate
        z     the position of the object: z coordinate
        id    the identifier of the object, e.g. ArUco id
        color the color of the object as string, e.g. "red"
        -------

        """
        obj = Class(namespace = self.get_onto())

        if id:
            obj.id = id

        if color:
            color = db.onto.NamedColor(color)
            obj.color.append(color)

        point = db.onto.Point2D(x=x, y=y, z=z)
        obj.location = point

        # TODO without this, some objects may be involuntarily destroyed by a garbage collector
        # TODO this should be investigated further to avoid some magical errors
        db.objects.append(obj)

        # set the internal attribute which signifies this is a real object in the workspace
        obj._is_in_workspace = True

        # obj._can_be_picked = True


    # TODO this method is intended to be used as an entry point to ROS node, the interface should be changed to work with ROS messages
    def update_object_position(self, id : str, x : float, y: float, z: float):
        """
        Updates the position of the existing object or adds the object if it does not exist yet.

        Parameters
        ----------
        id  the identifier of the object, e.g. ArUco id
        x   the position of the object: x coordinate
        y   the position of the object: y coordinate
        """
        obj = db.onto.search_one(id=id, _is_in_workspace=True)

        if not obj:
            # creates a new cube
            # TODO make it work with any object
            self.add_object(db.onto.Cube, x=x, y=y, z=z, id=id)
        else:
            obj.location.x = x
            obj.location.y = y
            obj.location.z = z


    def delete_object(self, obj : Any):
        """
        Deletes the object from the ontology.

        Parameters
        ----------
        obj  the object to be deleted
        """
        ow.destroy_entity(obj)


    def get_class_objects(self, cls: ClassVar) -> List:
        """
        Returns a list of real objects in the workspace with a certain class.

        Parameters
        ----------
        cls  the class of the objects
        """
        return db.onto.search(type=cls, _is_in_workspace=True)


    def get_all_tools(self) -> list:
        """
        Returns a list of all tools in the workspace.
        """
        return db.onto.search(type=db.onto.Tool, _is_in_workspace=True)


    def get_class_objects_by_color(self, cls: ClassVar, color: str) -> List:
        """
        Returns a list of real objects in the workspace with a certain class and a certain color.

        Parameters
        ----------
        cls   the class of the objects
        color the color of the objects as string
        """
        return db.onto.search(type=cls, color=db.onto.NamedColor(color), _is_in_workspace=True)


    def get_objects_by_color(self, color: str) -> List:
        """
        Returns a list of real objects in the workspace with a certain color.

        Parameters
        ----------
        color the color of the objects as string
        """
        return db.onto.search(color=db.onto.NamedColor(color), _is_in_workspace=True)


    def get_last_mentioned_object(self) -> Any:
        """
        Returns the object which was set as last mentioned or None if there is no such object.
        """
        return db.onto.search(_is_last_mentioned=True, _is_in_workspace=True).first()


    def set_last_mentioned_object(self, obj : Any):
        """
        Sets an object as last mentioned.

        Parameters
        ----------
        obj  the object which should be set as "last mentioned"
        """
        obj._is_last_mentioned = True


    def set_state(self, state : State):
        """
        Sets a current state of the program.

        Parameters
        ----------
        state  the state to be set
        """
        current_state = db.get_param("state")
        self.logger.debug(f"{current_state} => {state}")
        db.set_param("state", state)


    def set_param(self, key : str, val : Any):
        """
        Saves an arbitrary key-value pair to be retrieved later

        Parameters
        ----------
        key  the key of the parameter
        val  the value of the parameter
        """
        self.logger.debug(f"Setting parameter \"{key}\" to {val}")
        db.set_param(key, val)


    def get_state(self) -> State:
        """
        Returns current program state.
        """
        return db.get_param("state")

    def get_custom_templates(self):
        """
        Returns a list of custom templates learned during the learning phase.
        """
        custom_templates = db.onto.search(type=db.onto.RobotCustomProgram)

        #all_programs = db.onto.search(type=db.onto.RobotProgram)
        #custom_templates = filter(lambda x : x.name.startswith("$"), all_programs)

        return custom_templates

    def get_demonstration_templates(self):
        """
        Returns a list of templates learned during the demonstration.
        """
        demonstration_templates = db.onto.search(type=db.onto.RobotDemonstrationProgram)

        return demonstration_templates
    #
    # def get_picked_object(self):
    #     return onto.search_one(_is_picked=True, _is_in_workspace=True)

    def add_custom_template(self, template):
        """
        Adds a custom template to the ontology.

        Parameters
        ----------
        template  the template to be added
        """

        # program already exists in memory, just give it a unique name to distinguish it
        template.name = "$" + self.get_param("current_task_name")

        # TODO the above approach is not resistant to the garbage collector -> add also to the DB objects list
        db.objects.append(template)

    def save_program(self, program, name):
        """
        Adds a ungrounded/grounded program to be executed to the ontology.

        Parameters
        ----------
        program  the program to be added
        name     the name under which it is to be saved
        """

        # program already exists in memory, just give it a unique name to distinguish it
        program.name = name

    def add_area(self, area_name : str, corners : Tuple[db.onto.Location, db.onto.Location, db.onto.Location, db.onto.Location], is_default=False):
        area = db.onto.Area(name=area_name, default=is_default, _is_in_workspace=True)

        for corner in corners:
            area.corners.append(corner)

        db.objects.append(area)


    def get_default_area(self):
        default_area = self.get_by_properties(db.onto.Area, {"default" : True})

        if not default_area:
            self.logger.warning("Default area is not set.")

            x_min = int(Config.get("x_min", section="default_workspace"))
            x_max = int(Config.get("x_max", section="default_workspace"))
            y_min = int(Config.get("y_min", section="default_workspace"))
            y_max = int(Config.get("y_max", section="default_workspace"))

            return db.onto.Area(name="workspace", corners=[
                db.onto.Location(x=x_min, y=y_min, z=0),
                db.onto.Location(x=x_min, y=y_max, z=0),
                db.onto.Location(x=x_max, y=y_max, z=0),
                db.onto.Location(x=x_max, y=y_min, z=0),
            ])

        return default_area[0]


    def get_param(self, key : str):
        """
        Retrieves the value for the key in the database (previously set by set_param()) or None if the key does not exist.

        Parameters
        ----------
        key  the key of the parameter
        """
        return db.get_param(key)


    def get_by_properties(self, cls : ClassVar, properties : dict):
        """
        Returns a list of real objects in the workspace with a certain class and a certain properties
        (useful for grounding the placeholders).

        Parameters
        ----------
        cls         the class of the objects
        properties  a dictionary with ontology property (python) names as keys and
                    ontology objects (from the range of the respective property) as values
        -------

        """
        return db.onto.search(type=cls, **properties, _is_in_workspace=True)

    def get_onto(self) -> Any:
        """
        Returns the onto object to be used as a namespace
        """
        return db.onto

    def get_db(self):
        return db

    # def is_subclass_of(self, obj : ow.Thing, cls : ClassVar):
    #     is_a_list = obj.is_a
    #     subclasses = list(onto.search(subclass_of=cls))
    #
    #     return any([is_a in subclasses for is_a in is_a_list])

