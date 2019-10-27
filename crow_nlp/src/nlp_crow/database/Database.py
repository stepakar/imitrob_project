#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
from enum import Enum
from typing import Any

import owlready2 as ow
import os
from nlp_crow.utils.config import Config

class State(Enum):
    DEFAULT = 1
    LEARN_FROM_INSTRUCTIONS = 2
    DEFINE_AREA = 3
    LEARN_FROM_DEMONSTRATION = 4

class Database:
    """
    Acts as a singleton (the class can be instantiated only once during the run of the program).
    Should be accessed only via the DatabaseAPI class.
    """
    class __Database:
        pass

    instance = None

    def __init__(self, path = None):
        if not Database.instance:
            Database.instance = Database.__Database()

            onto_file_path = Config.get("onto_file_path")
            if path:
                print('loading database from path {}'.format(path))
                onto_file_path = path
            # dir = os.path.dirname(os.path.abspath(onto_file_path))
            # ow.onto_path.append(dir)

            # ow.default_world.set_backend(filename="file://" + onto_file_path, exclusive=False)
            self.onto = ow.get_ontology("file://" + onto_file_path).load()
            self.onto_ns = None
            self.params = {}

            # TODO see DatabaseAPI.add_object()
            self.objects = []

            self.set_param("state", State.DEFAULT)

        #else:
        #    Database.instance.val = arg

    def __getattr__(self, name ):
        return getattr(self.instance, name)

    def __setattr__(self, name, value):
        return setattr(self.instance, name, value)

    def set_param(self, key: str, value: Any):
        self.params[key] = value

    def get_param(self, key : str):
        return self.params.get(key)

    def change_onto(self, path):
        #ow.default_world.set_backend(filename="file://" + path, exclusive=False)
        if self.onto_ns is not None:
            # self.onto.__exit__()
            pass
        print(ow.onto_path)
        if path not in ow.onto_path:
            ow.onto_path.append(path)
        with open(path, 'rb') as file:
            self.onto = ow.get_ontology("file://" + path).load(fileobj=file, reload=True)
        # self.onto_ns = self.onto.__enter__()
        return self.onto

