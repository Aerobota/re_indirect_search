#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     re_indirect_search/learner.py
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

import numpy as np
from sklearn import mixture

import sklearn

if sklearn.__version__ < '0.11':
    raise ImportError('sklearn library has to be at least version 0.11.')


get_location = lambda pose: (pose[3], pose[7], pose[11])


class ContinuousGMMLearner(object):
    """ 
        Models relative location as a mixture of Gaussian
        This method is used to learn a mixture of Gaussian models of the
        distribution of relative locations between object pairs.

        Samples are NOT slices as opposed to MATLAB code.

        TODO: consider updating models incrementally, as new data arrives.
    """
    maxComponents = 5
    splitSize = 3
    minSample = np.ceil(maxComponents * splitSize / (splitSize - 1))

    def learn_batch(self, model):
        """ Learns the GMM probabilities.
            Loads the model if it exists otherwise runs the learning process.
        """
        print('Learning GMM Models ...')

        # Iterate through the relative location samples (dictionary)
        mixtures = model.get_evidence()
        num_of_mixtures = len(mixtures)
        for i, key in enumerate(mixtures):
            if len(mixtures[key])==0: #no samples
                continue

            model.add_mixture(key, Mixture(self._doGMM(mixtures[key], model.covariance_type), len(mixtures[key])))
            print('Learned parameters for the class pair: {key} ({current_mixture}/~{num_of_mixtures})'.format(key=key,current_mixture=str(i+1),num_of_mixtures=num_of_mixtures))

    def learn_one_sample(self, model, large_objects, small_objects):
        """ This is a method which implements the scenario where the locations
            of pairs of unknown small objects and known/unknown large object
            pairs are received and the task is to store their mean distance
            along with the previously learned models.
        """
        for large_object in large_objects:
            for small_object in small_objects:
                key = (large_object.type, small_object.type)

                if not model.has_mixture(key):                    
                    # learn from one sample and add it to model dictionary
                    dist = np.asarray(small_object.loc) - np.asarray(get_location(large_object.pose))
                    cylindrical_dist = [np.sqrt(dist[0] ** 2 + dist[1] ** 2), dist[2]]
                    mixture = Mixture(self._doGMM([cylindrical_dist], model.covariance_type), 1)
                    model.add_mixture(key, mixture)
                    print('Learned parameters for the class pair: {key}\n'
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
                               covariances=mixture.CLF.covars_))
                    

    def _doGMM(self, samples, covariance_type):
        """ Learns the GMM probabilities for the particular class pair i,j
            with samples containing distance information as a list of 2-d vectors.

            Learning is done with EM algorithm.
            Number of components is determined with the BIC-score.
            Restricting the number of components to MAXCOMPONENTS.

            Returns the learned model as a class.

            INPUT: Samples is an Nx2 numpy array containing evidence
            between a particular object pair.

            OUTPUT: CLF is a learned GMM model using the ML scikit-learn toolkit.

            TODO: use Dirichlet process instead of BIC score.
        """
        n_samples = len(samples)

        if n_samples > self.minSample:
            score = np.zeros(self.maxComponents)

            for test, train in self._split_crossvalidation(samples):
                # For every possible number of components calculate the score
                # and add the scores for all dataset splits
                for k in xrange(self.maxComponents):
                    score[k] = score[k] + self._evaluate_model_complexity(train, test, k + 1, covariance_type)

            # find the lowest cost
            kOPT = score.argmin() + 1

            # train model with optimal component size
            clf = mixture.GMM(n_components=kOPT, covariance_type=covariance_type)
            clf.fit(samples)
        else:
            kOPT = 1

            # initialize model without doing EM
            clf = mixture.GMM(n_components=kOPT, covariance_type=covariance_type)

            # if sample size is 1 we have to make up 2 more samples
            # very close to the original data point
            if n_samples == 1:
                eps = 0.1
                np_samples = np.asarray(samples)
                samples = np.vstack((np_samples, np_samples[0, :] - eps, np_samples[0, :] + eps))

            clf.fit(samples)

        # return the learned model
        return clf

    def _split_crossvalidation(self, samples):
        """ Generator for all possible combinations of dataset slices used for
            crossvalidation.

            The dataset is split into 'splitSize' parts; for each combination
            'splitSize - 1' parts are used for training and 1 for testing.
        """
        samples = np.random.permutation(samples)
        n_samples = len(samples)
        split = np.floor_divide(n_samples, self.splitSize)

        for indices in zip(xrange(0, n_samples - split, split),
                           xrange(split, n_samples, split)):
            train_l, test, train_u = np.split(samples, indices)

            if len(train_l) and len(train_u):
                yield test, np.vstack((train_l, train_u))
            elif len(train_l):
                yield test, train_l
            elif len(train_u):
                yield test, train_u
            else:
                raise ValueError('Can not split and concatenate samples')

    def _evaluate_model_complexity(self, trainSet, testSet, k, covariance_type):
        """ Evaluate the Bayesian Information Criterion (BIC) score
            of the GMM where the BIC is defined as:

            BIC = NlogN + m * log(n)

            NlogN: negative-log-likelihood of the test data in testSet
            m: estimated number of parameters
            n: number of data points

            Using Scikit-learn to implement GMM.
        """
        clf = mixture.GMM(n_components=k, covariance_type=covariance_type)

        # train with EM
        clf.fit(trainSet)
        bic = clf.bic(testSet)

        return bic


class Mixture(object):
    """ Wrapper for the CLF learned GMM model using the ML scikit-learn toolkit.
        Adds number of samples as a field, which is useful to indicate unsafe
        inferences.

        TODO: implement particular pickle method with __getstate__, __setstate__
    """
    numSamples = 0

    def __init__(self, clf, val):
        """ Initializes the wrapper class for the GMM models.
            The second argument is the sample size which is added as a field.
        """
        self.CLF = clf
        self.numSamples = val
