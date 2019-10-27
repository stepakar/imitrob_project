#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""

import os
import nlp_crow.utils.constants as c

def get_full_path(*path):
    """
    Retrieve the absolute path to the file based on the location of the project.
    """
    return os.path.join(c.PROJECT_MAIN_DIR, *path)