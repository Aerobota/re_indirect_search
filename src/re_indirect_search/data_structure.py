#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     re_indirect_search/data_structure.py
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
import scipy.io as sio

from zope.interface import implements

from re_indirect_search.interfaces import IDataSet, IImage, IObject


class NYUDataStructure(object):
    """
    """
    implements(IDataSet)

    DATASET_DIR = 'NYU_dataset'
    OBJECT_CATAGORIES = 'objectCategories'
    TEST_SET = 'groundTruthTest'
    TRAIN_SET = 'groundTruthTrain'

    def __init__(self, path, selection):
        """
        """
        self._path = os.path.join(path, self.DATASET_DIR)
        self._selection = selection

        if selection not in ['all', 'test', 'train']:
            raise ValueError("The argument 'selection' must be 'all', 'test', or 'train'.")

        self._init()

    def _init(self):
        """ Initialization routine of cache attributes separate such that it can
            be called from '__setstate__'.
        """
        self._images = ()
        self._classes = ()
        self._classesLarge = ()
        self._classesSmall = ()

    def __getstate__(self):
        return self._path, self._selection

    def __setstate__(self, state):
        self._path, self._selection = state

        self._init()

    @property
    def images(self):
        """ Tuple of all images occurring in the dataset. """
        if not self._images:
            self._loadDataMAT()

        return self._images

    @property
    def classNames(self):
        """ Return the names of all classes as a tuple of strings. """
        if not self._classes:
            self._loadClassesMAT()

        return self._classes

    @property
    def smallClassNames(self):
        """ Return the names of all small classes as a tuple of strings. """
        if not self._classesSmall:
            self._loadClassesMAT()

        return self._classesSmall

    @property
    def largeClassNames(self):
        """ Return the names of all large classes as a tuple of strings. """
        if not self._classesLarge:
            self._loadClassesMAT()

        return self._classesLarge

    def _loadDataMAT(self):
        """ Load the stored MAT file as a numpy array of Data objects.
        """
        images = []

        if self._selection == 'train':
            storageNames = (self.TRAIN_SET,)
        elif self._selection == 'test':
            storageNames = (self.TEST_SET,)
        else:
            storageNames = (self.TRAIN_SET, self.TEST_SET)

        for name in storageNames:
            path = os.path.join(self._path, name)

            try:
                mat = sio.loadmat(path, appendmat=True, squeeze_me=True)
            except IOError:
                print('File does not exist:', path)
                raise

            images.extend(mat['data'])

        self._images = tuple(Image(self._path, *image) for image in images)

    def _loadClassesMAT(self):
        """ Load the classes from MAT given by the field 'objectCatagories'.
        """
        path = os.path.join(self._path, self.OBJECT_CATAGORIES)

        try:
            mat = sio.loadmat(path, squeeze_me=True, appendmat=True)
        except IOError:
            print('File does not exist:', path)
            raise

        self._classes = tuple(str(name) for name in mat['names'])
        self._classesSmall = tuple(str(name) for name in mat['smallNames'])
        self._classesLarge = tuple(str(name) for name in mat['largeNames'])


class Image(object):
    """
    """
    implements(IImage)

    def __init__(self, path, img_name, depth_name, dir_path, img_size, calib, obj_path):
        """
        """
        self._path = path
        #self._img_name = str(img_name)
        #self._depth_name = str(depth_name)
        #self._dir_path = dir_path # No idea what this is exactly...
        #self._img_size = img_size
        #self._calib = calib
        self._obj_path = str(obj_path)

        self._objects = ()

    @property
    def objects(self):
        """ Tuple of all objects occurring in the image. """
        if not self._objects:
            self._loadObjectMAT()

        return self._objects

    def _loadObjectMAT(self):
        """ Loads the object MAT file associated with the particular image.

            Modifies the objectPath since the structure objects are now
            stored in 'py_object' and not in 'object'!
        """
        filename = os.path.join(self._path, 'py_{0}'.format(self._obj_path))

        try:
            mat = sio.loadmat(filename, squeeze_me=True)
        except IOError:
            print('File does not exist:', filename)
            raise

        self._objects = tuple(Object(*data) for data in mat['s'])


class Object(object):
    """
    """
    implements(IObject)

    def __init__(self, pos, dim, name, polygon):
        """
        """
        self._pos = pos
        #self._dim = dim
        self._name = str(name)
        #self._polygon = polygon # (type: struct)

    @property
    def pos(self):
        """ 3D postion of the object as a numpy array. """
        return self._pos

    @property
    def name(self):
        """ Name/descriptor of the object as a string. """
        return self._name


class SmallObject(object):
    """ Temporary class for to represent a small object for which is being
        queried. In the case of learning from one sample scenario, the location
        field has to be specified as well.
    """
    def __init__(self, obj_type, loc=None):
        self.type = obj_type

        if loc is not None:
            self.loc = loc
