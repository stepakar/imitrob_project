#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""

from nlp_crow.modules.CrowModule import CrowModule
from nlp_crow.structures.tagging.TaggedText import TaggedText
from nlp_crow.utils import nlp


class IdDetector(CrowModule):
    """
    Detects id of an object in text.
    """
    def __init__(self):
        pass

    def detect_id(self, text : TaggedText):
        """
        A simple method for detecting a color in text.
        Yes, there is some space for improvement.

        Parameters
        ----------
        text  an input text
        """
        try:
            idx = text.tokens.index("id")
            id_value = text.tokens[idx + 1]
        except ValueError:
            return None

        return str(nlp.get_int(id_value))