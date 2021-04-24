import numpy as np
import random
from math import sqrt, pi, exp

def gaussian_prob(obs, mu, sig):
    # Calculate Gaussian probability given
    # - observation
    # - mean
    # - standard deviation
    num = (obs - mu) ** 2
    denum = 2 * sig ** 2
    norm = 1 / sqrt(2 * pi * sig ** 2)
    return norm * exp(-num / denum)

# Gaussian Naive Bayes class
class GNB():
    # Initialize classification categories
    def __init__(self):
        self.classes = ['left', 'keep', 'right']

    # Given a set of variables, preprocess them for feature engineering.
    def process_vars(self, vars):
        # The following implementation simply extracts the four raw values
        # given by the input data, i.e. s, d, s_dot, and d_dot.
        s, d, s_dot, d_dot = vars
        return s, d, s_dot, d_dot

    # Train the GNB using a combination of X and Y, where
    # X denotes the observations (here we have four variables for each) and
    # Y denotes the corresponding labels ("left", "keep", "right").
    def train(self, X, Y):
        '''
        Collect the data and calculate mean and standard variation
        for each class. Record them for later use in prediction.
        '''

        label_temp = {}
        label_temp['left'] = []
        label_temp['keep'] = []
        label_temp['right'] = []

        for label in self.classes:
            for i in range(4):
                label_temp[label].append([])
        
        for i in range(len(X)):
            x = X[i]
            label = Y[i]

            x = self.process_vars(x)

            for index, val in enumerate(x):
                label_temp[label][index].append(val)
            
        mean = []
        std = []

        for i in self.classes:
            mean.append([])
            std.append([])

            for v in label_temp[i]:
                m = np.mean(v)
                s = np.std(v)

                mean[-1].append(m)
                std[-1].append(s)
        
        self.means = mean
        self.stds = std

        # TODO: implement code.

    # Given an observation (s, s_dot, d, d_dot), predict which behaviour
    # the vehicle is going to take using GNB.
    def predict(self, observation):
        '''
        Calculate Gaussian probability for each variable based on the
        mean and standard deviation calculated in the training process.
        Multiply all the probabilities for variables, and then
        normalize them to get conditional probabilities.
        Return the label for the highest conditional probability.
        '''
        # TODO: implement code.

        prob = []
        observation = self.process_vars(observation)
        for i in range(len(self.means)):
            m = self.means[i]
            s = self.stds[i]
            l = self.classes[i]

            product = 1

            for j in range(len(m)):
                mu = m[j]
                sig = s[j]
                o = observation[i]

                liklihood_temp = gaussian_prob(o,mu,sig)
                product = product * liklihood_temp
            prob.append(product)
        sum_prob = sum(prob)

        probs = [ p/sum_prob for p in prob]

        index = 0
        best = 0

        for i, p in enumerate(probs):
            if p > best:
                best = p
                index = 1
        names = ['left','keep','right']
        # return "keep"
        return names[index]

