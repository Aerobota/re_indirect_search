#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     re_indirect_search/evidence_generator.py
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
import itertools
import numpy as np

from zope.interface import implements

from re_indirect_search.interfaces import IEvidenceGenerator
from re_indirect_search.kb_translator import KBTranslator


get_location = lambda pose: (pose[3], pose[7], pose[11])




class EvidenceGenerator(object):
    '''
    Abstract base class
    This base class exists only for structure.
    '''
    largeObjectDefinitionsFile = 'largeObjectDefinitions.txt'

    def __init__(self, data_dir):
        """
        """
        self.data_dir = data_dir

        self._init()

    def _init(self):
        """ Initialization routine of cache attributes separate such that it can
            be called from '__setstate__'.
        """
        self.largeObjects = []

    def __getstate__(self):
        return self.data_dir

    def __setstate__(self, state):
        self.data_dir = state

        self._init()

    def loadLargeObjectDefinitions(self):
        '''
        Loads large objects.
        '''
        if not self.largeObjects:
            TRANSLATOR = KBTranslator(self.data_dir)
            self.largeObjects = [k for k, v in TRANSLATOR.large_obj.iteritems()]

        #why return?
        return self.largeObjects


class LocationEvidenceGenerator(EvidenceGenerator):
    '''
    Produces location Evidence
    This class is an abstract class that produces evidence of relative
    and absolute locations of objects.
    '''
    implements(IEvidenceGenerator)

    def get_position_evidence(self, objects):
        """ objects: IObject
        """
        raise NotImplementedError

    def get_relative_evidence(self, a, b):
        """
        """
        raise NotImplementedError

    def get_evidence(self, data_set):
        """ Produces relative location evidence.

            DATASTR is a DATAHANDLER.DATASTRUCTURE class instance
            containing the location data.

            RETURNS: EVIDENCE is a dictionary where EVIDENCE[(object_i,object_j)]
            contains the samples from class i to class j.
            The format of the samples depends on the implementation of
            GETRELATIVEEVIDENCE in the derived class.
        """
        evidence = {classPair : np.zeros((0, 2))
                    for classPair in itertools.product(data_set.classNames, repeat=2)}

        # go through each room scanning for evidence
        for image in data_set.images:
            objs = image.objects
            pos = self.get_position_evidence(objs)
            relEvidence = self.get_relative_evidence(pos, pos)
            names = tuple(obj.name for obj in objs)

            #storing also the distance of small-small and large-large object occurrences
            for i in range(len(names)):
                for j in range(len(names)):
                    try:
                        evidence[(names[i], names[j])] = \
                        np.vstack((evidence[(names[i], names[j])], relEvidence[i, j, :]))
                    except KeyError:
                        print 'This should not happen!'
                        raise

        return evidence

    def get_evidence_for_semantic_map(self, semMap, epsilon, delta):
        '''
        Produces relative location evidence for the mesh generated for
        the semantic map.
        Observed objects: Large classes

        SEMMAP is the semantic map received.

        EVIDENCE is a dictionary with two keys:
        'names': the class names of the observed objects
        'absEvidence': the 3D-location of the mesh generated
        'relEvidence': the relative location from every observed object
        to every pixel
        '''
        objs = semMap.objects
        classesLarge = self.loadLargeObjectDefinitions()

        evidence = dict()
        evidence['names'] = list()
        idx = list()

        # get the types of objects in the semantic map
        for i, c in enumerate(objs):
            if c.type in classesLarge:
                idx.append(i)
                evidence['names'].append(c.type)

        # positions of large objects
        objPos = np.zeros((3, len(idx)))
        for i, val in enumerate(idx):
            objPos[:, i] = np.array(get_location(objs[val].pose))

        evidence['absEvidence'] = self._generate_mesh(objPos, epsilon, delta)
        evidence['relEvidence'] = self.get_relative_evidence(objPos, evidence['absEvidence'])

        return evidence

    def _generate_mesh(self, obj_pos, epsilon, delta):
        """ Generates a mesh for the partial point cloud of the semantic map
            SEMMAP for which evidence was collected through large objects.

            OBJPOS is the 3d-positions of the large objects in the scene.

            Generates a box surrounding the large objects where the edges are
            at least EPSILON meters away from the objects vertices. The mesh is
            equidistant points inside this cube, each point DELTA away from
            each other.
        """
        mins = obj_pos.min(axis=1) - epsilon
        maxs = obj_pos.max(axis=1) + epsilon

        x = np.arange(mins[0], maxs[0] + delta, delta)
        y = np.arange(mins[1], maxs[1] + delta, delta)
        z = np.arange(mins[2], maxs[2] + delta, delta)

        return self._cartesian([x, y, z]).transpose()

    def _cartesian(self, arrays, out=None):
        """ Generate a cartesian product of input arrays.
            TODO: replace with np.mgrid for mesh generation.

            Parameters
            ----------
            arrays : 1-D numpy arrays to form the cartesian product of.
            out : ndarray
                Array to place the cartesian product in.

            Returns
            -------
            out : ndarray
                2-D array of shape (M, len(arrays)) containing cartesian products
                formed of input arrays.

            Examples
            --------
            >>> cartesian(([1, 2, 3], [4, 5], [6, 7]))
            array([[1, 4, 6],
                   [1, 4, 7],
                   [1, 5, 6],
                   [1, 5, 7],
                   [2, 4, 6],
                   [2, 4, 7],
                   [2, 5, 6],
                   [2, 5, 7],
                   [3, 4, 6],
                   [3, 4, 7],
                   [3, 5, 6],
                   [3, 5, 7]])

            @snippet Code taken from:
            http://stackoverflow.com/questions/1208118/using-numpy-to-build-an-array-of-all-combinations-of-two-arrays
        """
        dtype = arrays[0].dtype

        n = np.prod([x.size for x in arrays])

        if out is None:
            out = np.zeros([n, len(arrays)], dtype=dtype)

        m = n / arrays[0].size
        out[:, 0] = np.repeat(arrays[0], m)
        if arrays[1:]:
            self._cartesian(arrays[1:], out=out[0:m, 1:])
            for j in xrange(1, arrays[0].size):
                out[j * m:(j + 1) * m, 1:] = out[0:m, 1:]

        return out


class CylindricalEvidenceGenerator(LocationEvidenceGenerator):
    '''
    Produces relative locations in cylindrical coordinates
    This class implements Learner.LocationEvidenceGenerator. The
    returned evidence is 2-dimensional where the first dimension is the
    horizontal distance and the second the height.
    '''
    def get_position_evidence(self, objs):
        '''
        Return the positions of each object in the image
        as a matrix of column stacked 3d-positions.
        '''
        return np.vstack(obj.pos for obj in objs).transpose()

    def get_relative_evidence(self, sourcePos, targetPos):
        '''
        Returns cylindrical evidence as a 3D-array of
        1. Object-to-object distance matrix in xz-coordinates (radius)
        2. Object-to-object matrix of vertical distances (y).
        '''
        # initialize dist array
        num_source_obj = np.shape(sourcePos)[1]
        num_target_obj = np.shape(targetPos)[1]
        dist = np.zeros((num_source_obj, num_target_obj, 3))
        evidence = np.zeros((num_source_obj, num_target_obj, 2))

        # Broadcasting vec here
        for d in range(3):
            vec = targetPos[d, :]
            mat = vec[:, np.newaxis] - sourcePos[d, :]
            dist[:, :, d] = mat.transpose()

        evidence[:, :, 0] = np.sqrt(dist[:, :, 1] ** 2 + dist[:, :, 2] ** 2)
        evidence[:, :, 1] = dist[:, :, 0]

        return evidence
