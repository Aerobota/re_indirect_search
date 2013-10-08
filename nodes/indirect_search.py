#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     nodes/indirect_search.py
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

import os.path

import roslib; roslib.load_manifest('re_indirect_search')
from rospkg import RosPack
import rospy
import numpy

from geometry_msgs.msg import Point
from re_indirect_search.srv import InferenceQuery, LearnQuery, InferenceQueryResponse, LearnQueryResponse

from re_indirect_search.model import ModelError, GMMModel
from re_indirect_search.data_structure import SmallObject
from re_indirect_search.learner import ContinuousGMMLearner
from re_indirect_search.kb_translator import KBTranslator

## SET PARAMETERS
PKG_PATH = RosPack().get_path('re_indirect_search')
DATA_PATH = os.path.join(PKG_PATH, 'data')
MODEL_PATH = os.path.join(DATA_PATH, 'GMMFull.bin')

MAX_DISTANCE = 0.5
STRETCH = 0.4          # the amount by which the mesh is stretched
GRID_RESOLUTION = 0.06 # fineness of the grid [m]

MODEL = GMMModel()
TRANSLATOR = KBTranslator(DATA_PATH)

try:
    MODEL.load(MODEL_PATH)
except ModelError as e:
    print(e)
    exit()


def learn(req):
    resp = LearnQueryResponse()

    # reformat request
    sem_map = req.map
    known_large_objects, known_small_objects = TRANSLATOR.split_large_and_small_objects(sem_map.objects)

    if not known_large_objects or not known_small_objects:
        resp.status = False
        return resp

    # Hard coding for safety # TODO: Remove this by adding a check on known keys
    learning_candidates = ['breakfastcereal']
    

    final_candidates = [obj for obj in known_small_objects if obj.type in learning_candidates]

    if not final_candidates:
        resp.status = False
        return resp

    # TODO: Remove this specific definition of SmallObject
    small_objects = [SmallObject(obj.type, [obj.pose[3], obj.pose[7], obj.pose[11]]) for obj in final_candidates]

    # run query
    learner = ContinuousGMMLearner()
    learner.learn_one_sample(MODEL, known_large_objects, small_objects)

    resp.status = True
    return resp


def infer(req):
    resp = InferenceQueryResponse()
    
    # reformat request
    sem_map = req.map
    sem_map.objects, _ = TRANSLATOR.split_large_and_small_objects(sem_map.objects) # throwing away all small objects and all unknown large objects from sem_map.objects
    
    if not sem_map.objects: #No Known big objects
        print('No Known big objects!')
        resp.status = False
        return resp

    _, small_objs = TRANSLATOR.split_large_and_small_objects([SmallObject(req.query_object)]) #req.query_object is a KnowRob type

    if not small_objs:
        print('Query object type unknown!')
        resp.status = False
        return resp

    # run query
    candidates = MODEL.infer(sem_map, small_objs[0], STRETCH, GRID_RESOLUTION, MAX_DISTANCE)

    # reformat response
    resp = InferenceQueryResponse()

    if candidates is None:
        resp.status = False
    else:
        resp.status = True

        for candidate in candidates:
            # return only the locations that has a probability higher than some tresh hold.
            if candidate.prob > 0.01:
                resp.locations.append(Point(candidate.pos[0],
                                            candidate.pos[1],
                                            candidate.pos[2]))
                resp.probabilities.append(candidate.prob)
    
    # check if the response contains at least one candidate point
    if len(resp.locations)>0:    
        resp.probabilities = resp.probabilities/numpy.sum(resp.probabilities)
        return resp
    else:
        print 
        resp.status = False
        return resp

def main():
    rospy.init_node('indirect_search')
    rospy.Service('learn', LearnQuery, learn)
    rospy.Service('infer', InferenceQuery, infer)
    rospy.spin()


if __name__ == '__main__':
    main()
