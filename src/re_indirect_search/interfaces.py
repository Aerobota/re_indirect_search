#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     re_indirect_search/interfaces.py
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

from zope.interface import Interface, Attribute


class IEvidenceGenerator(Interface):
    """
    """
    def get_evidence(data_set): #@NoSelf
        """
        """

    def get_evidence_for_semantic_map(sem_map, epsilon, delta): #@NoSelf
        """
        """


class IDataSet(Interface):
    """
    """
    images = Attribute('Tuple of all images occurring in the dataset. (type: IImage)')
    classNames = Attribute('Tuple of the names of all classes. (type: str)')
    smallClassNames = Attribute('Tuple of the names of all small classes. (type: str)')
    largeClassNames = Attribute('Tuple of the names of all large classes. (type: str)')


class IImage(Interface):
    """
    """
    objects = Attribute('Tuple of all objects occurring in the image. (type: IObject)')


class IObject(Interface):
    """
    """
    pos = Attribute('3D postion of the object. (type: numpy.array)')
    name = Attribute('Name/descriptor of the object. (type: str)')
