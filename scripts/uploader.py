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

from mod_semantic_map.msg import SemMap, SemMapObject

from re_indirect_search.model import GMMModel
from re_indirect_search.data_structure import NYUDataStructure
from re_indirect_search.evidence_generator import CylindricalEvidenceGenerator
from re_indirect_search.kb_translator import KBTranslator


## SET PARAMETERS
PKG_PATH = RosPack().get_path('re_indirect_search')
DATA_PATH = os.path.join(PKG_PATH, 'data')
MODEL_PATH = os.path.join(DATA_PATH, 'GMMFull.bin')


def uploader(sleepTime):
    model = GMMModel()
    evidence_generator = CylindricalEvidenceGenerator(DATA_PATH);
    model.init(evidence_generator,
               NYUDataStructure(DATA_PATH, 'all'))
    translator = KBTranslator(DATA_PATH)

    pub = rospy.Publisher('NYUSemMap', SemMap)

    print('Getting ready to go through the full dataset...')
    frameID = 0

    for scene in model._data_set.images:
        print('Processing frameID# - {0}'.format(frameID))

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

        #publish the generate SemMap
        pub.publish(sem_map)

        frameID += 1
        rospy.sleep(sleepTime)


if __name__ == '__main__':
    rospy.init_node('evidence_uploader')

    try:
        sleepTime = float(sys.argv[1])
    except:
        sleepTime = 1.0; # default interval between topics

    uploader(sleepTime)
