#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     scripts/learner.py
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

#importing the necessary modules for learning
from re_indirect_search.model import ModelError, GMMModel
from re_indirect_search.data_structure import NYUDataStructure
from re_indirect_search.learner import ContinuousGMMLearner
from re_indirect_search.evidence_generator import CylindricalEvidenceGenerator


## SET PARAMETERS
PKG_PATH = RosPack().get_path('re_indirect_search')
DATA_PATH = os.path.join(PKG_PATH, 'data')
MODEL_PATH = os.path.join(DATA_PATH, 'GMMFull.json')


def clean(args):
    os.rename(MODEL_PATH, MODEL_PATH + '.bak')


def list_objects(args):
    print('NOT IMPLEMENTED!')
    print('list object:', args.type)


def list_data(args):
    print('NOT IMPLEMENTED!')
    print('list data:', args.large_object, args.small_object)


def dump(args):
    model = GMMModel(CylindricalEvidenceGenerator(DATA_PATH), NYUDataStructure(DATA_PATH, 'train'))

    try:
        model.load(MODEL_PATH)
    except ModelError as e:
        print(e)
        return

    with open(args.filename, 'w') as f:
        f.write('Brief summary of the GMM Models:\n\n')
        f.write(model.get_log())


def learn(args):
    model = GMMModel(CylindricalEvidenceGenerator(DATA_PATH), NYUDataStructure(DATA_PATH, 'train'),'diag')
    #model.init(CylindricalEvidenceGenerator(DATA_PATH), NYUDataStructure(DATA_PATH, 'train')) # set to all, during final demo

    learner = ContinuousGMMLearner()
    learner.learn_batch(model)

    model.save(MODEL_PATH)


def _get_argparser():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='learner.py')
    subparsers = parser.add_subparsers() # TODO: help='???'

    # clean
    clean_parser = subparsers.add_parser('clean', help='Delete learned models')
    clean_parser.set_defaults(func=clean)

    # list objects
    list_parser = subparsers.add_parser('list', help='List objects')
    list_parser.add_argument('type', choices=['all', 'large', 'small'],
                             nargs='?', default='all',
                             help='Select type of objects to list')
    list_parser.set_defaults(func=list_objects)

    # list data
    data_parser = subparsers.add_parser('data', help='List data for object pair')
    data_parser.add_argument('large_object', help='Select type of large object')
    data_parser.add_argument('small_object', help='Select type of small object')
    data_parser.set_defaults(func=list_data)

    # dump
    dump_parser = subparsers.add_parser('dump', help='Dump GMM model to a text file')
    dump_parser.add_argument('filename', help='Filename of text file')
    dump_parser.set_defaults(func=dump)

    # learn
    learn_parser = subparsers.add_parser('learn', help='Learn the models')
    learn_parser.set_defaults(func=learn)

    return parser


def main():
    args = _get_argparser().parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
