#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     re_indirect_search/kbTranslator.py
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
import ConfigParser, os
from mod_semantic_map.msg import SemMap, SemMapObject

class kbTranslator(object):
    """ Model has to be either initialized or loaded!
    """
    def __init__(self, DATA_PATH):
        """Create two dicts (large, small objects translation)
        """
        _config = ConfigParser.ConfigParser()
        _config.read(os.path.join(DATA_PATH, 'objectDefinitions.txt'))
        self.smallObjDict = self.ConfigSectionMap(_config, 'small_objects')
        self.largeObjDict = self.ConfigSectionMap(_config, 'large_objects')
        self.objDict = dict(self.largeObjDict.items()+self.smallObjDict.items())
        self.invLargeObjDict = self.inverseDict(self.largeObjDict) #Assuming one-to-one mapping
        self.invSmallObjDict = self.inverseDict(self.smallObjDict) #Assuming one-to-one mapping
        
    def ConfigSectionMap(self, config, section):
        dict1 = {}
        options = config.options(section)
        for option in options:
            try:
                dict1[option] = config.get(section, option)
                if dict1[option] == -1:
                    dict1[option] = ''
            except:
                dict1[option] = ''
            #Create your own definition if no definition is given or if there is something wrong
            if dict1[option] == '': dict1[option] = 'http://ias.cs.tum.edu/kb/knowrob.owl#'+ option
        return dict1
        
    def inverseDict(self, dict):
        return {v:k for k,v in dict.items()}
    
    def split_into_known_large_and_small_objects(self, semMap):
        
        known_small_objects = []
        known_large_objects = []
        for object in semMap.objects:
            try:
                object.type=self.invLargeObjDict[object.type]
                known_large_objects.append(object)
            except:
                try:
                    object.type=self.invSmallObjDict[object.type]
                    known_small_objects.append(object)
                except:
                    pass
                
        return [known_large_objects,known_small_objects]
    
    def translate_KBtoNYU_small_objects(self, KBtype):

        try:
            NYUtype = self.invSmallObjDict[KBtype]
            return NYUtype
        except:
            return ''
    
    
        
        
        
        

