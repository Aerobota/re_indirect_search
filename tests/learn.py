#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     tests/learn.py
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

from geometry_msgs.msg import Point
from mod_semantic_map.msg import SemMap, SemMapObject
from re_indirect_search.srv import LearnQuery, InferenceQuery


def init_map(x, y, z):
    sem_map = SemMap()
    sem_map.header.frame_id = "http://www.example.com/foo.owl#"

    obj = SemMapObject()
    obj.id = 1
    obj.partOf = 0
    obj.type = 'table'
    obj.pose = [1.0, 0.0, 0.0, x,
                0.0, 1.0, 0.0, y,
                0.0, 0.0, 1.0, z,
                0.0, 0.0, 0.0, 1.0]
    sem_map.objects.append(obj)

    return sem_map


def main():
    try:
        rospy.wait_for_service('learn')
    except rospy.ROSException:
        print("Timeout while waiting for ROS service 'learn'.")
        return

    print('Run Learn query...')
    learn = rospy.ServiceProxy('learn', LearnQuery)
    learn(init_map(0.0, 0.0, 0.0), 'serial box', Point(0.5, 0.4, 0.3))

    try:
        rospy.wait_for_service('infer')
    except rospy.ROSException:
        print("Timeout while waiting for ROS service 'infer'.")
        return

    print('Run Inference query...')
    infer = rospy.ServiceProxy('infer', InferenceQuery)
    candidates = infer(init_map(0.6, 0.6, 0.6), 'serial box')

    if not candidates.status:
        print('Inference query could not be processed.')
        return

    print('Result:')
    for pos, prob in zip(candidates.locations, candidates.probabilities):
        print '  [{0}, {1}, {2}] - {3}'.format(pos.x, pos.y, pos.z, prob)


if __name__ == '__main__':
    main()
