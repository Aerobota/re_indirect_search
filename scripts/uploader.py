#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     scripts/uploader.py
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

import sys
import os.path

import roslib; roslib.load_manifest('re_indirect_search')
from rospkg import RosPack
import rospy

from mod_semantic_map.srv import GenerateSemanticMapOWL
from mod_semantic_map.msg import SemMap, SemMapObject
from re_srvs.srv import SetEnvironment

from re_indirect_search.model import GMMModel
from re_indirect_search.data_structure import NYUDataStructure
from re_indirect_search.evidence_generator import CylindricalEvidenceGenerator
from re_indirect_search.kb_translator import KBTranslator



## SET PARAMETERS
PKG_PATH = RosPack().get_path('re_indirect_search')
DATA_PATH = os.path.join(PKG_PATH, 'data')
MODEL_PATH = os.path.join(DATA_PATH, 'GMMFull.bin')




def looper(uploader, converter):
    model = GMMModel()
    evidence_generator = CylindricalEvidenceGenerator(DATA_PATH);
    model.init(evidence_generator,
               NYUDataStructure(DATA_PATH, 'all'))
    translator = KBTranslator(DATA_PATH)

    ## Fixed Values
    cls = 'NYU_Depth_Dataset_V2' #Environment Class
    id_head = 'Scene_ID' #Environment ID
    description = ('A scene from NYU Depth Dataset V2.'+ 
                   'For more details see http://cs.nyu.edu/~silberman/datasets/nyu_depth_v2.html.'+
                   ' For the related software see http://github.com/IDSCETHZurich/re_indirect_search.git')
        
    environment = 'owl_string'
    apiKey = '6e616a616758cf1e985a387853dbb32b4c9bc683203864ae3c'
    files = []
    
    frameID = 1

    for scene in model._data_set.images:
        print('Image name - {0}'.format(scene._obj_path))
        
        print('Processing frameID# - {0}'.format(frameID))
        #tmp break for debugging
        if frameID > 1:
            break

        objs = scene.objects
        pos = evidence_generator.get_position_evidence(objs)

        sem_map = SemMap()
        sem_map.header.frame_id = 'NYU-DATASET#{0}'.format(frameID)
        sem_map.header.stamp = rospy.get_rostime()

        for i, name in enumerate(obj.name for obj in objs):
            sem_obj = SemMapObject()

            try:
                sem_obj.type = translator.obj[name]
            except KeyError:
                sem_obj.type = name

            sem_obj.pose = [1.0, 0.0, 0.0, pos[0][i],
                            0.0, 1.0, 0.0, pos[1][i],
                            0.0, 0.0, 1.0, pos[2][i],
                            0.0, 0.0, 0.0, 1.0]

            sem_map.objects.append(sem_obj)

        #convert the generate SemMap to owl
        try:
            converter_response = converter(sem_map)
        except rospy.ServiceException as e:
            print('Service call failed: {0}'.format(e))
        
        try:
            uploader_response = uploader(cls, id_head+str(frameID), description, converter_response.owlmap, files, apiKey)
        except rospy.ServiceException as e:
            print('Service call failed: {0}'.format(e))
        
        if uploader_response.success:
            print('frameID# - {0}: Upload successful.'.format(frameID))
        else:
            print('frameID# - {0}: Upload failed.'.format(frameID))


        frameID += 1


if __name__ == '__main__':
    rospy.init_node('evidence_uploader')
    
    print('Uploader script starting...')
    print('waiting for the genrate_owl_map service...')
    rospy.wait_for_service('/knowrob_semantic_map_to_owl/generate_owl_map')
    print('waiting for re_comm/set_environment')
    rospy.wait_for_service('re_comm/set_environment')

    uploader = rospy.ServiceProxy('re_comm/set_environment', SetEnvironment)
    converter = rospy.ServiceProxy('/knowrob_semantic_map_to_owl/generate_owl_map', GenerateSemanticMapOWL)
    
    looper(uploader, converter)
    
