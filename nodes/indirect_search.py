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

import os.path

import roslib; roslib.load_manifest('re_indirect_search')
from rospkg import RosPack
import rospy
import numpy as np

from geometry_msgs.msg import Point
from re_indirect_search.srv import InferenceQuery, LearnQuery, InferenceQueryResponse, LearnQueryResponse

from re_indirect_search.model import ModelError, GMMModel
from re_indirect_search.data_structure import SmallObject
from re_indirect_search.learner import ContinuousGMMLearner
from re_indirect_search.kb_translator import KBTranslator
from re_indirect_search.evidence_generator import CylindricalEvidenceGenerator
from re_indirect_search.data_structure import NYUDataStructure

## SET PARAMETERS
PKG_PATH = RosPack().get_path('re_indirect_search')
DATA_PATH = os.path.join(PKG_PATH, 'data')
MODEL_PATH = os.path.join(DATA_PATH, 'GMMFull.json')

MAX_DISTANCE = 1.2    # 
STRETCH = 2.0         # the amount by which the mesh is stretched, this should be larger than any mean of the model
GRID_RESOLUTION = 0.1 # fineness of the grid [m] # typical value 0.1 [m]
MAX_CANDIDATES = 10

MODEL = GMMModel(CylindricalEvidenceGenerator(DATA_PATH), NYUDataStructure(DATA_PATH, 'train'),'diag') #last arg. is the covariance_type
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
    probs,points = MODEL.infer(sem_map, small_objs[0], STRETCH, GRID_RESOLUTION, MAX_DISTANCE)
    # The above is ordered list (probability descending)
    
    resp = InferenceQueryResponse()
    if points is None:
        resp.status = False
    else:
        resp.status = True
        finalIndex = []
        totalProb = 0.0
        
        length = len(probs)
        if length>MAX_CANDIDATES:
            length = MAX_CANDIDATES

        resp.locations = points[0:length]
        totProb = np.sum(probs[0:length])
        resp.pointProbabilities = [probs[i]/totProb for i in range(length)]   
        
        ##    fill in big objects and corresponding probabilities    ##
        resp.bigObjectIDs = [obj.id for obj in sem_map.objects]            
        
        resp.bigObjectProbability = np.zeros(len(resp.bigObjectIDs))
        tmp = np.zeros(len(resp.bigObjectIDs))
        
        i = 0
        for pos, prob in zip(resp.locations, resp.pointProbabilities):
            
            for index, obj in enumerate(sem_map.objects):
                tmp[index] = (obj.pose[3]-pos.x)**2 + (obj.pose[7]-pos.y)**2 + (obj.pose[11]-pos.z)**2

            resp.bigObjectProbability[np.argmin(tmp)] += prob
            print(' {0}: object index={1}, prob={2} dist={3}'.format(i,np.argmin(tmp), prob, tmp))
            i=i+1

    return resp

def main():
    rospy.init_node('indirect_search')
    rospy.Service('learn', LearnQuery, learn)
    rospy.Service('infer', InferenceQuery, infer)
    rospy.spin()


if __name__ == '__main__':
    main()
