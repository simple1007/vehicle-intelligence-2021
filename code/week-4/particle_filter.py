import numpy as np
from helpers import distance
import math
class ParticleFilter:
    def __init__(self, num_particles):
        self.initialized = False
        self.num_particles = num_particles

    # Set the number of particles.
    # Initialize all the particles to the initial position
    #   (based on esimates of x, y, theta and their uncertainties from GPS)
    #   and all weights to 1.0.
    # Add Gaussian noise to each particle.
    def initialize(self, x, y, theta, std_x, std_y, std_theta):
        self.particles = []
        for i in range(self.num_particles):
            self.particles.append({
                'x': np.random.normal(x, std_x),
                'y': np.random.normal(y, std_y),
                't': np.random.normal(theta, std_theta),
                'w': 1.0,
                'assoc': [],
            })
        self.initialized = True

    # Add measurements to each particle and add random Gaussian noise.
    def predict(self, dt, velocity, yawrate, std_x, std_y, std_theta):
        # Be careful not to divide by zero.
        v_yr = velocity / yawrate if yawrate else 0
        yr_dt = yawrate * dt
        for p in self.particles:
            # We have to take care of very small yaw rates;
            #   apply formula for constant yaw.
            if np.fabs(yawrate) < 0.0001:
                xf = p['x'] + velocity * dt * np.cos(p['t'])
                yf = p['y'] + velocity * dt * np.sin(p['t'])
                tf = p['t']
            # Nonzero yaw rate - apply integrated formula.
            else:
                xf = p['x'] + v_yr * (np.sin(p['t'] + yr_dt) - np.sin(p['t']))
                yf = p['y'] + v_yr * (np.cos(p['t']) - np.cos(p['t'] + yr_dt))
                tf = p['t'] + yr_dt
            p['x'] = np.random.normal(xf, std_x)
            p['y'] = np.random.normal(yf, std_y)
            p['t'] = np.random.normal(tf, std_theta)

    # Find the predicted measurement that is closest to each observed
    #   measurement and assign the observed measurement to this
    #   particular landmark.
    def associate(self, predicted, observations):
        associations = []
        # For each observation, find the nearest landmark and associate it.
        #   You might want to devise and implement a more efficient algorithm.
        for o in observations:
            min_dist = -1.0
            for p in predicted:
                dist = distance(o, p)
                if min_dist < 0.0 or dist < min_dist:
                    min_dist = dist
                    min_id = p['id']
                    min_x = p['x']
                    min_y = p['y']
            
            association = {
                'id': min_id,
                'x': min_x,
                'y': min_y,
            }
            # o['id']=min_id
            associations.append(association)
            # o['id'] = min_id
        # Return a list of associated landmarks that corresponds to
        #   the list of (coordinates transformed) predictions.
        return associations

    # Update the weights of each particle using a multi-variate
    #   Gaussian distribution.
    def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks):
        associations = []
        # sense_x = []
        # sense_y = []
        norm_weight = 0.0
        observation = []
        for p in self.particles:
            prt_x = p['x']
            prt_y = p['y']
            prt_theta = p['t'] 

            predictions = []
            transformed_obs = []

            for k,v in map_landmarks.items():
                x_ml = v['x']
                y_ml = v['y']
                if distance(p,v) < sensor_range:
                # if np.fabs(x_ml - prt_x) <= sensor_range and np.fabs(y_ml - prt_y) <= sensor_range:
                    # print(distance(p,v))
                    id_ = k
                    predictions.append({'id':id_,'x':v['x'],'y':v['y']})
                # print(predictions)
                # print(distance(p,v))

            for obs in observations:
                obs_temp = {}
                obs_temp['x'] = prt_x + (obs['x'] * math.cos(prt_theta)) - obs['y'] * math.sin(prt_theta)
                obs_temp['y'] = prt_y + (obs['x'] * math.sin(prt_theta)) + obs['y'] * math.cos(prt_theta)
                transformed_obs.append(obs_temp)
                
                # print(obs_temp['x'])
                # print(obs_temp['y'])
                # print('-----------')
            
            if len(predictions) == 0:
                continue

            p['w'] = 1.0
            
            association_temp = self.associate(predictions,transformed_obs)
            
            for i in association_temp:
                normalizer = 1.0/(2.0*math.pi*std_landmark_x*std_landmark_y)
                
                exponent = ((p['x']-i['x']) ** 2) / (2*(std_landmark_x**2)) + ((p['y']-i['y']) ** 2) / (2 * (std_landmark_y**2))
                # print(exponent)
                
                obs_w = normalizer * math.exp(-1.0 * exponent)  
                obs_w += 1.e-300 # avoid round-off to zero
                p['w'] *= obs_w
                associations.append(i['id'])
            
            p['assoc'] = associations

        
        # TODO: For each particle, do the following:
        # 1. Select the set of landmarks that are visible
        #    (within the sensor range).
        # 2. Transform each observed landmark's coordinates from the
        #    particle's coordinate system to the map's coordinates.
        # 3. Associate each transformed observation to one of the
        #    predicted (selected in Step 1) landmark positions.
        #    Use self.associate() for this purpose - it receives
        #    the predicted landmarks and observations; and returns
        #    the list of landmarks by implementing the nearest-neighbour
        #    association algorithm.
        # 4. Calculate probability of this set of observations based on
        #    a multi-variate Gaussian distribution (two variables being
        #    the x and y positions with means from associated positions
        #    and variances from std_landmark_x and std_landmark_y).
        #    The resulting probability is the product of probabilities
        #    for all the observations.
        # 5. Update the particle's weight by the calculated probability.

        # pass

    # Resample particles with replacement with probability proportional to
    #   their weights.
    
    def resample(self):
        # return
        # TODO: Select (possibly with duplicates) the set of particles
        #       that captures the posteior belief distribution, by
        # 1. Drawing particle samples according to their weights.
        # 2. Make a copy of the particle; otherwise the duplicate particles
        #    will not behave independently from each other - they are
        #    references to mutable objects in Python.
        # Finally, self.particles shall contain the newly drawn set of
        #   particles.
        import random

        weights = [p['w'] for p in self.particles] 
        resampled_particles = []
        N = len(weights)
        positions = (np.arange(N) + np.random.random()) / N
    
        indexes = np.zeros(N, 'i')
        cumulative_sum = np.cumsum(weights)
        i, j = 0, 0
        while i < N and j<N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1
        
        resampled_particles.append(self.particles[indexes[i]])

        self.particles = resampled_particles


    # Choose the particle with the highest weight (probability)
    def get_best_particle(self):
        highest_weight = -1.0
        for p in self.particles:
            if p['w'] > highest_weight:
                highest_weight = p['w']
                best_particle = p
        return best_particle
