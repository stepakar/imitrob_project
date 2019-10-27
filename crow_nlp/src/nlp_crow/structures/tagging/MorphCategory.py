#!/usr/bin/env python
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
import logging

import owlready2 as ow

from nlp_crow.database.Database import Database

db = Database()

# with db.onto as onto:

class MorphCategory(ow.Thing):
    """
    A superclass for single morphological category (e.g. a POS tag).
    """
    namespace = db.onto
    values = None
    alt_lookup_table = {}

    def __init__(self, value: str, **kwargs):
        super().__init__(**kwargs)
        self.logger = logging.getLogger(__name__)

        id, description = self.decode(value)

        self.id = id
        self.description = description

    def decode(self, value : str):
        id = self.alt_lookup_table.get(value.lower(), None)

        if not id:
            self.logger.warning(f"Morphological category {value} not recognized")
            return "<unk>", "<unk>"

        cat = self.values[id]
        name = cat["name"]

        return id, name

    def __str__(self):
        return self.id

    def __eq__(self, other):
        if self.__class__ != other.__class__:
            return False

        return self.id == other.id

    @classmethod
    def calculate_alt_lookup_table(cls, values):
        # calculates a mapping string -> category id for every possible representation of the category
        # e.g. for the "NN" category: { "NN" : "NN", "N" : "NN", "noun" : "NN" }
        alt_lookup_table = {value.lower(): key for (key, value_list) in values.items() for value in
                            value_list["alt"]}

        return alt_lookup_table


# TODO check if these classes are saved in ontology and delete the code
class hasId(MorphCategory >> str, ow.FunctionalProperty):
    namespace = db.onto
    python_name = "id"

class hasDescription(MorphCategory >> str, ow.FunctionalProperty):
    namespace = db.onto
    python_name = "description"


class POS(MorphCategory):
    namespace = db.onto
    values = {
        "CC": {"alt": ["CC"], "name": "coordinating conjunction"},
        "CD": {"alt": ["CD"], "name": "cardinal digit"},
        "DT": {"alt": ["DT"], "name": "determiner"},
        "EX": {"alt": ["EX"], "name": "existential there"},
        "FW": {"alt": ["FW"], "name": "foreign word"},
        "HERE": {"alt": ["HERE"], "name": "here"},
        "IN": {"alt": ["IN"], "name": "preposition/subordinating conjunction"},
        "JJ": {"alt": ["JJ"], "name": "adjective 'big'"},
        "JJR": {"alt": ["JJR"], "name": "adjective, comparative 'bigger'"},
        "JJS": {"alt": ["JJS"], "name": "adjective, superlative 'biggest'"},
        "LS": {"alt": ["LS"], "name": "list marker 1)"},
        "MD": {"alt": ["MD"], "name": "modal could, will"},
        "NN": {"alt": ["NN"], "name": "noun, singular 'desk'"},
        "NNS": {"alt": ["NNS"], "name": "noun plural 'desks'"},
        "NNP": {"alt": ["NNP"], "name": "proper noun, singular 'Harrison'"},
        "NNPS": {"alt": ["NNPS"], "name": "proper noun, plural 'Americans'"},
        "PDT": {"alt": ["PDT"], "name": "predeterminer 'all the kids'"},
        "POS": {"alt": ["POS"], "name": "possessive ending parent's"},
        "PRP": {"alt": ["PRP"], "name": "personal pronoun I, he, she"},
        "PRP$": {"alt": ["PRP"], "name": "possessive pronoun my, his, hers"},
        "RB": {"alt": ["RB"], "name": "adverb very, silently"},
        "RBR": {"alt": ["RBR"], "name": "adverb, comparative better"},
        "RBS": {"alt": ["RBS"], "name": "adverb, superlative best"},
        "RL": {"alt": ["RL"], "name": "relation"},
        "RP": {"alt": ["RP"], "name": "particle give up"},
        "TO": {"alt": ["TO"], "name": "to go 'to' the store."},
        "UH": {"alt": ["UH"], "name": "interjection errrrrrrrm"},
        "VB": {"alt": ["VB"], "name": "verb, base form take"},
        "VBD": {"alt": ["VBD"], "name": "verb, past tense took"},
        "VBG": {"alt": ["VBG"], "name": "verb, gerund/present participle taking"},
        "VBN": {"alt": ["VBN"], "name": "verb, past participle taken"},
        "VBP": {"alt": ["VBP"], "name": "verb, sing. present, non-3d take"},
        "VBZ": {"alt": ["VBZ"], "name": "verb, 3rd person sing. present takes"},
        "WDT": {"alt": ["WDT"], "name": "wh-determiner which"},
        "WP": {"alt": ["WP"], "name": "wh-pronoun who, what"},
        "WP$": {"alt": ["WP"], "name": "possessive wh-pronoun whose"},
        "WRB": {"alt": ["WRB"], "name": "wh-abverb where, when"},
    }

    # values = {
    #     'A': {"alt" : ["A", "Adjective", "ADJ"], "name" :"Adjective",  "desc" : "Adjective"},
    #     'C': {"alt" : ["C", "Numeral", "NUM"], "name" :"Numeral",  "desc" : "Numeral"},
    #     'D': {"alt" : ["D", "Adverb", "ADV"], "name" :"Adverb",  "desc" : "Adverb"},
    #     'I': {"alt" : ["I", "Interjection", "INTJ"], "name" :"Interjection",  "desc" : "Interjection"},
    #     'J': {"alt" : ["J", "Conjunction", "CONJ", "CCONJ", "conjuction"], "name" :"Conjunction", "desc" : "Conjunction"},
    #     'N': {"alt" : ["N", "Noun", "NOUN", "substantive"], "name" :"Noun", "desc" : "Noun"},
    #     'P': {"alt" : ["P", "Pronoun", "PRON", "pronomina"], "name" :"Pronoun", "desc" : "Pronoun"},
    #     'V': {"alt" : ["V", "Verb", "VERB"], "name" : "Verb", "desc" : "Verb"},
    #     'R': {"alt" : ["R", "Preposition", "ADP"], "name" : "Preposition", "desc" : "Preposition"},
    #     'T': {"alt" : ["T", "Particle", "PART"], "name" : "Particle", "desc" : "Particle"},
    #     'X': {"alt" : ["X", "Unknown"], "name" : "Unknown", "desc" : "Unknown"},
    #     'Z': {"alt" : ["Z", "Punctuation", "PUNCT", "SYM"], "name" : "Punctuation", "desc" : "Punctuation"}
    # }

    alt_lookup_table = MorphCategory.calculate_alt_lookup_table(values)