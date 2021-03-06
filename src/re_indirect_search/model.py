#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     re_indirect_search/model.py
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

try:
    import cPickle as pickle
except ImportError:
    import pickle

import jsonpickle
import simplejson
from ast import literal_eval

import numpy as np
from sklearn import mixture

from zope.interface.verify import verifyObject
from geometry_msgs.msg import Point

from re_indirect_search.interfaces import IEvidenceGenerator, IDataSet



class ModelError(Exception):
    """
    """


class GMMModel(object):
    """ Model has to be either initialized or loaded!
    """
    def __init__(self, evidenceGenerator, dataSet, covariance_type):
        """
        """
        self._evidence_generator = evidenceGenerator
        self._data_set = dataSet
        self.covariance_type = covariance_type
        self._model = {}
        

#     def init(self, evidence_generator, data_set):
#         """
#         """
#         if self._model:
#             raise ModelError('Model data is already initialized.')
# 
#         verifyObject(IEvidenceGenerator, evidence_generator)
#         verifyObject(IDataSet, data_set)
# 
#         self._evidence_generator = evidence_generator
#         self._data_set = data_set
#         self._model = {}

    def load(self, model_file):
        """ Loads the GMM models.
        """
        if self._model:
            raise ModelError('Model data is already initialized.')

        try:
            with open(model_file, 'rb') as f:
                print('Loading GMM Models...'),
                json_unpickled_models = jsonpickle.decode(f.read())
        except IOError:
            raise ModelError('Model could not be loaded. File does not exist.')
        
        #TODO: Move this stuff to the class?
        models = {literal_eval(key):value for key, value in json_unpickled_models.items()}
    
        for key, value in models.items():
            oldCLF = models[key].CLF
            n_components = literal_eval(models[key].CLF.n_components)
            clf = mixture.GMM(n_components=n_components, covariance_type=self.covariance_type)
            
            models[key].CLF = clf 
            models[key].CLF.covars_ = np.array(literal_eval(oldCLF.covars_.replace('array','')))
            models[key].CLF.means_ = np.array(literal_eval(oldCLF.means_.replace('array','')))
            models[key].CLF.weights_ = np.array(literal_eval(oldCLF.weights_.replace('array','')))
        
        self._model = models #models_keys_fixed 
        #print self._model
        print 'Done'

    def save(self, model_file):
        """ Saves the GMM models.
        """
        if not self._model:
            raise ModelError('No model data available.')

        #data = (self._model, self._evidence_generator, self._data_set)

        try:
            with open(model_file,'w') as f:
                json_pickle_output = jsonpickle.encode(self._model,max_depth=3)
                f.write(simplejson.dumps(simplejson.loads(json_pickle_output), indent=4))
        except IOError:
            raise ModelError('Model could not be saved.')

    def get_log(self):
        """ Get a nicely formatted text detailing the model.
        """
        return '\n\n'.join('Learned parameters for the class pair: {key}\n'
                           'Number of components: {components}\n'
                           'Number of samples: {samples}\n'
                           'Weights:\n{weights}\n'
                           'Means:\n{means}\n'
                           'Covariances:\n{covariances}'.format(
                               key=key,
                               components=mixture.CLF.n_components,
                               samples=mixture.numSamples,
                               weights=mixture.CLF.weights_,
                               means=mixture.CLF.means_,
                               covariances=mixture.CLF.covars_)
                           for key, mixture in self._model.iteritems())

    def has_mixture(self, object_pair):
        """ Callback for Learner to check if there is already a mixture
            registered for the given object pair.
        """
        return object_pair in self._model

    def add_mixture(self, object_pair, mixture):
        """ Callback for Learner to register a newly learned mixture for the
            given object pair.
        """
        self._model[object_pair] = mixture

    def get_evidence(self):
        """
        """
        return self._evidence_generator.get_evidence(self._data_set)
    
    def _print_mixture_info(self, key, mixture, additionalText='No additional text'):
        """
        For debugging
        """
        
        print additionalText 
        print 'Learned parameters for the class pair: {key}\n'\
               'Number of components: {components}\n'\
               'Number of samples: {samples}\n'\
               'Weights:\n{weights}\n'\
               'Means:\n{means}\n'\
               'Covariances:\n{covariances}'.format(
                   key=key,
                   components=mixture.CLF.n_components,
                   samples=mixture.numSamples,
                   weights=mixture.CLF.weights_,
                   means=mixture.CLF.means_,
                   covariances=mixture.CLF.covars_)

    def infer(self, sem_map, small_object, epsilon, delta, max_distance):
        '''
        This method infers the location of queried objects SMALLOBJECTS
        using SEMMAP. SMALLOBJECTS is a list of small object strings.

        Returns a dictionary CANDIDATEPOINTS
        which has small objects as keys and list of CANDIDATEPOINT structures
        as values.
        '''
        # Get the probability distributions over the scenes'
        # point cloud and the location of each point in the cloud.
        probVec, locVec = self._probabilities_for_semantic_map(sem_map, small_object, epsilon, delta)

        return self._get_candidate_points(probVec, locVec, max_distance)

    def _probabilities_for_semantic_map(self, sem_map, small_object, epsilon, delta):
        '''
        [PROBVEC,LOCVEC] = PROBABILITIESFORSEMMAP(SEMMAP,LOCLEARNER,SMALLOBJ)
        Returns the probability and position of each point in the scenes point
        cloud.

        The scene's point cloud is just the box containing the locations
        of the large objects pushed further by a certain threshold EPSILON.
        These are fields of the class originally set by the constructor.

        SEMMAP is an implementation of a SemMap structure containing
        SemMapObject list.

        LOCATIONLEARNER is an implementation of a Learner.LocationLearner.

        SMALLOBJECTS is a list of small objects queried for.

        PROBVEC is a dictionary of dictionaries
        with keys same as SMALLOBJECTS and OBSERVED OBJECTS
        where every entry is a vector of probabilities for every point in the cloud.

        LOCVEC is a 3xn numpy array where each column is the 3D-position of
        a point of the cloud.

        '''
        evidence = self._evidence_generator.get_evidence_for_semantic_map(sem_map, epsilon, delta) 
        
        '''
        The above Generates evidence['relEvidence'], evidence['absEvidence']
        relEvidence: for each large object in the semantic map
        absEvidence: one set
        '''

        probVec = dict()
        totalSamples = 0
        # for each (observed) large object
        for idx_o in range(evidence['relEvidence'].shape[0]):  
            name_large_obj = evidence['names'][idx_o]
            if name_large_obj == 'unknown': continue
#             # make sure object has unique identifier
#             ind = 1; o_ind = o
#             while o_ind in probVec:
#                 o_ind = o + str(ind)
#                 ind = ind + 1
            # each row in mat should correspond to a single data point
            mat = np.squeeze(evidence['relEvidence'][idx_o, :, :])
            key = (name_large_obj, small_object.type)
            mixture =  self._model[key]

            #probVec[name_large_obj] = np.exp(mixture.CLF.score(mat)) * mixture.numSamples
            probVec[idx_o] = np.exp(mixture.CLF.score(mat)) * mixture.numSamples
            totalSamples = totalSamples + mixture.numSamples
            
            # Debugging
            self._print_mixture_info(key,mixture,'_probabilities_for_semantic_map')
        try:
            # Compute the mean of the pairwise probabilities
            # TODO: do a mean based on the sample sizes
            
            probVec['mean'] = np.sum(probVec.values(), 0) / (len(probVec.values())*totalSamples)
            # normalize probVec['mean']
            
        except ZeroDivisionError:
            #observed large objects are apparently not in dataset
            # make a uniform distribution
            size = evidence['absEvidence'].shape[1]
            probVec['mean'] = 1 / size * np.ones((1, size))
        
        # the absolute locations were returned with the evidence dictionary
        locVec = evidence['absEvidence']
        
        return probVec, locVec

    def _get_candidate_points(self, probVec, locVec, maxDistance):
        '''
        [INRANGE,CANDIDATEPROB,CANDIDATEPOINTS]=GETCANDIDATEPOINTS(PROBVEC,LOCVEC,TRUEPOS,MAXDISTANCE)
        Generates candidatePoints.

        PROBVEC is a single field(i.e. class) from the struct returned
        by Evaluator.LocationEvaluator.probabilityVector.

        LOCVEC is the array of the same name returned as well by
        Evaluator.LocationEvaluator.probabilityVector.

        MAXDISTANCE is the scalar maximum distance in metres that a
        candidate point can have to a ground truth object and be
        counted as inRange.

        RETURNS a list of CANDIDATEPOINT structure where:

        CANDIDATEPROB denotes the probability of each
        candidate point.

        CANDIDATEPOS is a 3-vector showing position of the candidate point

        See also EVALUATOR.LOCATIONEVALUATOR.PROBABILITYVECTOR

        @attention: Unlike MATLAB code, here running a for loop to
        calculate range for candidate points
        '''
        # sort the point cloud by decreasing probability
        probs = probVec['mean']
        ind = probs.argsort()
        probs = probs[:, ind]
        locVec = locVec[:, ind]
        
        reducedProbs = []
        reducedPoints = []
        
        # list of candidate points
        candidatePoints = list()
        while locVec.shape[1] is not 0:
            # Fit a candidate point to the location of highest
            # probability
            reducedProbs.append(probs[-1])
            reducedPoints.append(Point(locVec[0, -1],locVec[1,-1], locVec[2,-1])) #geometry_msgs/Point
            # Remove all cloud points in range of the new point
            probs, locVec = self._remove_covered_points(probs, locVec, locVec[:,-1], maxDistance)
        
        # return all points without removing any
#         for i, loc in enumerate(locVec):
#             candidatePoints.append(CandidatePoint(probs[i],loc))
        return reducedProbs, reducedPoints

    def _remove_covered_points(self, probVec, locVec, pos, maxDistance):
        '''
        Removes all points inside maxDistance of the candidate point
        @attention: Unlike matlab candPoint is passed as a structure
        where pos-field is needed.
        '''
        # get 2nd dimension of locVec
        num = locVec.shape[1]
        pos = pos[:, np.newaxis]
        dist = (pos * np.ones((3, num))) - locVec
        pointsOutside = (sum(dist * dist) > (maxDistance * maxDistance))
        
        locVec = locVec[:, pointsOutside]
        probVec = probVec[:, pointsOutside]
        
        return probVec, locVec
    