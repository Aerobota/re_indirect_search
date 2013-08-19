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
#

import os.path

import roslib; roslib.load_manifest('re_indirect_search')
from rospkg import RosPack
import rospy

from geometry_msgs.msg import Point
from re_indirect_search.srv import InferenceQuery, LearnQuery, InferenceQueryResponse, LearnQueryResponse

from re_indirect_search.model import ModelError, GMMModel
from re_indirect_search.data_structure import SmallObject
from re_indirect_search.learner import ContinuousGMMLearner
from re_indirect_search.kbTranslator import kbTranslator


## SET PARAMETERS
PKG_PATH = RosPack().get_path('re_indirect_search')
DATA_PATH = os.path.join(PKG_PATH, 'data')
MODEL_PATH = os.path.join(DATA_PATH, 'GMMFull.bin')

MAX_DISTANCE = 1.0
STRETCH = 0.5          # the amount by which the mesh is stretched
GRID_RESOLUTION = 0.05 # fineness of the grid [m]

MODEL = GMMModel()
translator = kbTranslator(DATA_PATH)

try:
    MODEL.load(MODEL_PATH)
except ModelError as e:
    print(e)
    exit()


def learn(req):
    resp = LearnQueryResponse()
    
    # reformat request
    [known_large_objects,known_small_objects] = translator.split_into_known_large_and_small_objects(req.map)
    if len(known_large_objects)==0 or len(known_small_objects)==0:
        resp.status=false
        return resp
    
    req.map.objects=known_large_objects
    
    #Hardcoding for safety #TODO: Remove this by adding a check on known keys
    leariningCandidates = ['breakfastcereal']
    
    print known_small_objects
    
    finalCandidates=[]
    for small_object in known_small_objects:
         if small_object.type in leariningCandidates:
             finalCandidates.append(small_object)
    
    if len(finalCandidates)==0:
        resp.status=False
        return resp  
    
     
    
    small_objs = []
    for small_obj in finalCandidates:
        #SmallObject definition is really unnecessary
        small_objs.append(SmallObject(small_obj.type,[small_obj.pose[3], small_obj.pose[7], small_obj.pose[11]]))
    

    # run query
    learner = ContinuousGMMLearner()
    learner.learn_one_sample(MODEL, req.map, small_objs)

    resp.status=True
    return resp


def infer(req):
    resp = InferenceQueryResponse()
    
    sem_map = req.map
    sem_map.objects = translator.keep_only_known_large_objects_and_translate_KBtoNYU(req.map)[0]
    if len(sem_map.objects) == 0: #No Known big objects
        print 'No Known big objects!'
        resp.status = False
        return resp
    
    smallObjType = translator.translate_KBtoNYU_small_objects(req.query_object)
    if smallObjType=='':
        print 'Query object type unknown!'
        resp.status = False
        return resp

    small_objs = [SmallObject(smallObjType)]
    
    # run query
    candidates = MODEL.infer(sem_map, small_objs, STRETCH, GRID_RESOLUTION, MAX_DISTANCE)

    # reformat response
    resp = InferenceQueryResponse()

    if small_objs[0] not in candidates:
        resp.status = False
    else:
        resp.status = True

        for candidate in reversed(candidates[small_objs[0]]):
            resp.locations.append(Point(candidate.pos[0],
                                        candidate.pos[1],
                                        candidate.pos[2]))
            resp.probabilities.append(candidate.prob)

    return resp


def main():
    rospy.init_node('indirect_search')
    rospy.Service('learn', LearnQuery, learn)
    rospy.Service('infer', InferenceQuery, infer)
    rospy.spin()


if __name__ == '__main__':
    main()
