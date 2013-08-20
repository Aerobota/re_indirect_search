#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     re_indirect_search/kb_translator.py
#
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#
#     Copyright 2013 RoboEarth
#
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#
#

import os.path
import ConfigParser


class KBTranslator(object):
    """
    """
    def __init__(self, data_path):
        """Create two dicts (large, small objects translation)
        """
        config = ConfigParser.ConfigParser()
        config.read(os.path.join(data_path, 'objectDefinitions.txt'))

        self.large_obj = self._config_section_map(config, 'large_objects')
        self.small_obj = self._config_section_map(config, 'small_objects')

        self.obj = dict(self.large_obj.items() + self.small_obj.items())

        #Assuming one-to-one mapping for inverted dictionaries
        self.inv_large_obj = {v : k for k, v in self.large_obj.iteritems()}
        self.inv_small_obj = {v : k for k, v in self.small_obj.iteritems()}

    def _config_section_map(self, config, section):
        mapping = {}

        for option in config.options(section):
            mapping[option] = config.get(section, option)

            if not mapping[option]:
                #Create your own definition if no definition is given
                mapping[option] = 'http://ias.cs.tum.edu/kb/knowrob.owl#{0}'.format(option)

        return mapping

    def split_large_and_small_objects(self, objects):
        """
        """
        known_small_objects = []
        known_large_objects = []

        for obj in objects:
            obj_type = self.inv_large_obj.get(obj.type)

            if obj_type:
                obj.type = obj_type
                known_large_objects.append(obj)
                continue

            obj_type = self.inv_small_obj.get(obj.type)

            if obj_type:
                obj.type = obj_type
                known_small_objects.append(obj)

        return known_large_objects, known_small_objects
