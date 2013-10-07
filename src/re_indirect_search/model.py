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
#
#

try:
    import cPickle as pickle
except ImportError:
    import pickle

import numpy as np

from zope.interface.verify import verifyObject

from re_indirect_search.interfaces import IEvidenceGenerator, IDataSet


class ModelError(Exception):
    """
    """


class GMMModel(object):
    """ Model has to be either initialized or loaded!
    """
    def __init__(self):
        """
        """
        self._evidence_generator = None
        self._data_set = None
        self._model = None

    def init(self, evidence_generator, data_set):
        """
        """
        if self._model:
            raise ModelError('Model data is already initialized.')

        verifyObject(IEvidenceGenerator, evidence_generator)
        verifyObject(IDataSet, data_set)

        self._evidence_generator = evidence_generator
        self._data_set = data_set
        self._model = {}

    def load(self, model_file):
        """ Loads the GMM models.
        """
        if self._model:
            raise ModelError('Model data is already initialized.')

        try:
            with open(model_file, 'rb') as f:
                print('Loading GMM Models...')
                data = pickle.load(f)
        except IOError:
            raise ModelError('Model could not be loaded. File does not exist.')

        self._model, self._evidence_generator, self._data_set = data

    def save(self, model_file):
        """ Saves the GMM models.
        """
        if not self._model:
            raise ModelError('No model data available.')

        data = (self._model, self._evidence_generator, self._data_set)

        try:
            with open(model_file, 'wb') as f:
                print('Saving GMM Models...')
                pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)
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
        probVec, locVec = self._probabilities_for_semantic_map(sem_map, small_object, delta, epsilon)

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

        probVec = dict()

        # For each (small object) class and observed object
        # compute the pairwise probability

        probVec = dict()

        # for each (observed) large object
        for idx_o in range(evidence['relEvidence'].shape[0]):
            o = evidence['names'][idx_o]
            if o == 'unknown': continue
            # make sure object has unique identifier
            ind = 1; o_ind = o
            while o_ind in probVec:
                o_ind = o + str(ind)
                ind = ind + 1
            # each row in mat should correspond to a single data point
            mat = np.squeeze(evidence['relEvidence'][idx_o, :, :])
            probVec[o_ind] = np.exp(self._model[(o, small_object.type)].CLF.score(mat))
        try:
            # Compute the mean of the pairwise probabilities
            probVec['mean'] = np.sum(probVec.values(), 0) / len(probVec.values())
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

        # list of candidate points
        candidatePoints = list()
        while locVec.shape[1] is not 0:
            # Fit a candidate point to the location of highest
            # probability
            newCand = CandidatePoint(probs[0], locVec[:, 0])
            candidatePoints.append(newCand)
            # Remove all cloud points in range of the new point
            probs, locVec = self._remove_covered_points(probs, locVec, newCand, maxDistance)

        return candidatePoints

    def _remove_covered_points(self, probVec, locVec, candPoint, maxDistance):
        '''
        Removes all points inside maxDistance of the candidate point
        @attention: Unlike matlab candPoint is passed as a structure
        where pos-field is needed.
        '''
        # get 2nd dimension of locVec
        num = locVec.shape[1]
        pos = candPoint.pos[:, np.newaxis]
        dist = (pos * np.ones((3, num))) - locVec
        pointsOutside = (sum(dist * dist) > (maxDistance * maxDistance))

        locVec = locVec[:, pointsOutside]
        probVec = probVec[:, pointsOutside]

        return probVec, locVec


class CandidatePoint(object):
    '''
    Used to represent the candidate points
    '''
    def __init__(self, prob, locVec):
        self.prob = prob
        self.pos = locVec
