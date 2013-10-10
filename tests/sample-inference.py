#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     tests/inference.py
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

import roslib; roslib.load_manifest('re_indirect_search')
import rospy

from mod_semantic_map.msg import SemMap, SemMapObject
from re_indirect_search.srv import InferenceQuery


def init_map():
    sem_map = SemMap()
    sem_map.header.frame_id = 'http://www.example.com/foo.owl#'

    obj = SemMapObject()
    obj.id = 1
    obj.partOf = 0
    obj.type = 'http://ias.cs.tum.edu/kb/knowrob.owl#Cabinet-PieceOfFurniture'
    obj.type = 'http://ias.cs.tum.edu/kb/knowrob.owl#Table-PieceOfFurniture'
    obj.depth = 1
    obj.width = 1
    obj.height = 1
    obj.pose = [1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0]
    #sem_map.objects.append(obj)

    obj = SemMapObject()
    obj.id = 1
    obj.partOf = 0
    obj.type = 'http://ias.cs.tum.edu/kb/knowrob.owl#Bed-PieceOfFurniture'
    obj.depth = 1
    obj.width = 1
    obj.height = 1
    obj.pose = [1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0]
    sem_map.objects.append(obj)

    return sem_map


def main():
    print('Waiting for the inference service...')

    try:
        rospy.wait_for_service('infer')
    except rospy.ROSException:
        print("Timeout while waiting for ROS service 'infer'.")
        return

    print('Run Inference query...')
    infer = rospy.ServiceProxy('infer', InferenceQuery)
    candidates = infer(init_map(), 'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingBottle')

    if not candidates.status:
        print('Inference query could not be processed.')
        return

    print('Result:')
    for pos, prob in zip(candidates.locations, candidates.probabilities):
        print('  [{0}, {1}, {2}] - {3}'.format(pos.x, pos.y, pos.z, prob))


if __name__ == '__main__':
    main()
