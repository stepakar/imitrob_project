#!/usr/bin/env python
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
from nlp_crow.database.Database import Database
from nlp_crow.structures.tagging.MorphCategory import POS


import owlready2 as ow

db = Database()

# with db.onto as onto:
class Tag(ow.Thing):
    """
    Represents positional tags e.g. as defined here - http://ufal.mff.cuni.cz/pdt2.0/doc/manuals/en/m-layer/html/ch02s02s01.html
    TODO: extend for other tags than POS
    """
    namespace = db.onto
    def __str__(self):
        return f"{self.pos}"

    def equals(self, other, include_pos_subcategories : bool = True):
        # TODO make it work with all categories

        if include_pos_subcategories:
            return self.pos.id.startswith(other.pos.id)

        return self.pos.id == other.pos.id


# TODO check if this class is saved in ontology and delete the code
class hasPos(Tag >> POS, ow.FunctionalProperty):
    namespace = db.onto
    python_name = "pos"