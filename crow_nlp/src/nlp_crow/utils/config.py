#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""

from configparser import ConfigParser

from nlp_crow.utils.files import get_full_path

class Config:
    """
    Wrapper for ConfigParser. Provides access to project parameters in config/config.ini.
    """
    config_parser = ConfigParser()
    config_file_path = get_full_path('config', 'config.ini')
    is_initialized = False

    @classmethod
    def initialize(cls):
        cls.config_parser.read(cls.config_file_path)
        cls.is_initialized = True

    @classmethod
    def get(cls, key, section="global"):
        if not cls.is_initialized:
            cls.initialize()

        return cls.config_parser.get(section, key)
