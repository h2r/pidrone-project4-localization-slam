#!/usr/bin/env python
"""""
global_position_estimator_distance

Implements Monte-Carlo Localization for the PiDrone
"""""

import math
import numpy as np
import cv2


# ---------- map parameters ----------- #
MAP_PIXEL_WIDTH = 3227  # in pixel
MAP_PIXEL_HEIGHT = 2447
MAP_REAL_WIDTH = 1.4  # in meter
MAP_REAL_HEIGHT = 1.07
# -------------------------- #

# ----- camera parameters DO NOT EDIT ----- #
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)
CAMERA_SCALE = 290.
METER_TO_PIXEL = (float(MAP_PIXEL_WIDTH) / MAP_REAL_WIDTH + float(MAP_PIXEL_HEIGHT) / MAP_REAL_HEIGHT) / 2.
# ----------------------------- #

# ----- keyframe parameters ----- #
KEYFRAME_DIST_THRESHOLD = CAMERA_HEIGHT - 40
KEYFRAME_YAW_THRESHOLD = 0.175 # 10 degrees
# -------------------------- #

# ----- feature parameters DO NOT EDIT ----- #
ORB_GRID_SIZE_X = 4
ORB_GRID_SIZE_Y = 3
MATCH_RATIO = 0.7
MIN_MATCH_COUNT = 10
MAP_GRID_SIZE_X = ORB_GRID_SIZE_X * 3
MAP_GRID_SIZE_Y = ORB_GRID_SIZE_Y * 3
CELL_X = float(MAP_PIXEL_WIDTH) / MAP_GRID_SIZE_X
CELL_Y = float(MAP_PIXEL_HEIGHT) / MAP_GRID_SIZE_Y
PROB_THRESHOLD = 0.001
ORB_FEATURES_PER_GRID_CELL = 500
# -------------------------- #


class Particle(object):
    """"
    each particle holds poses and weights of all particles
    z is currently not used
    """""

    def __init__(self, i, poses, weights):
        self.i = i
        self.poses = poses
        self.weights = weights
    #return the weight 
    def weight(self): return self.weights[self.i]
    #return the pose x,y,z  and yaw with 4 functions
    def x(self): return #pose x 

    def y(self): return #pose y

    def z(self): return #pose z

    def yaw(self): return #pose yaw

    def __str__(self):
        return str(self.x()) + ' , ' + str(self.y()) + ' weight ' + str(self.weight())

    def __repr__(self):
        return str(self.x()) + ' , ' + str(self.y()) + ' weight ' + str(self.weight())


class ParticleSet(object):
    
    #make a class called particle set that takes an object and sets the 
    # weight, particle list, poses and num_particles

    def __init__(self, num_particles, poses):
        self.weights = #todo
        self.particles = #todo
        self.poses = #todo
        self.num_particles = #todo 


class LocalizationParticleFilter:
    """
    Particle filter for localization.
    """

    def __init__(self, map_kp, map_des):
        #todo 
        self.map_kp = 
        self.map_des = 

        self.particles = 
        self.measure_count = 

        index_params = 
        search_params = 
        self.matcher = 
        self.previous_time = 

        self.key_kp = 
        self.key_des = 

        self.z = 
        self.angle_x = 
        self.angle_y = 

        self.sigma_x = 
        self.sigma_y = 
        self.sigma_yaw = 

        sigma_vx = 
        sigma_vy = 
        sigma_vz = 
        sigma_yaw = 
        self.covariance_motion = np.array([[sigma_vx ** 2, 0, 0, 0],
                                           [0, sigma_vy ** 2, 0, 0],
                                           [0, 0, sigma_vz ** 2, 0],
                                           [0, 0, 0, sigma_yaw ** 2]])

    def update(self, z, angle_x, angle_y, prev_kp, prev_des, kp, des):
        """
        We implement the MCL algorithm from probabilistic robotics (Table 8.2)
        kp is the position of detected features
        des is the description of detected features
        """
        # update parameters
        self.z = 
        self.angle_x = 
        self.angle_y = 

        transform = self.compute_transform(prev_kp, prev_des, kp, des)

        if transform is not None:

            #todo 
            x = 
            y =
            # CCW rotation makes yaw more positive
            yaw = #todo 

            self.sample_motion_model(x, y, yaw)

            # if there is some previous keyframe
            if self.key_kp is not None and self.key_des is not None:

                transform = #todo 

                if transform is not None:
                    # distance since previous keyframe in PIXELS
                    x = #todo 
                    y = #todo 
                    yaw = #todo 

                    # if we've moved an entire camera frame distance since the last keyframe (or yawed 10 degrees)
                    if #todo 

                        #call the meaurement model and define key_kp and key_des
                        
                else:
                    # moved too far to transform from last keyframe, so set a new one
                   

            # there is no previous keyframe
            else:
               

        self.resample_particles()
        return self.get_estimated_position()

    def sample_motion_model(self, x, y, yaw):
        """
        Implement motion model from Equation 3 in PiDrone Slam with noise.
        """
        # add noise 
        noisy_x_y_z_yaw = np.random.multivariate_normal() #todo 

        for i in range(self.particles.num_particles):
            pose = 
            old_yaw = pose[3]
            #q1
            pose[0] +=
            pose[1] += 
            pose[2] = 
            pose[3] += 
            pose[3] = 

    def measurement_model(self, kp, des):
        """
        landmark_model_known_correspondence from probablistic robotics 6.6
        """
        for i in range(self.particles.num_particles):
            position = self.particles.poses[i]

            # get grid x and y for position 0
            

            # get grid x and y for position 1
           

            # get 8 map cells around pose
            sub_map_kp = self.map_kp[grid_x][grid_y]
            sub_map_des = self.map_des[grid_x][grid_y]

            # measure current global position
            pose, num = 

            # compute weight of particle
            if pose is None:
                q = #todo
            else:
                # add noise
                noisy_pose = 

                noisy_pose[3] = 

                yaw_difference = 
                #adjust the angle of the yaw difference
                yaw_difference = 

                # norm_pdf(x, 0, sigma) gets you the probability of x
                q = 

            # keep floats from overflowing
            self.particles.weights[i] = max(q, PROB_THRESHOLD)

    def resample_particles(self):
        """""
        samples a new particle set, biased towards particles with higher weights
        """""
        weights_sum = np.sum(self.particles.weights)
        new_poses = []
        new_weights = []
        #todo 
        normal_weights =  # normalize
        # samples sums to num_particles with same length as normal_weights, positions with higher weights are
        # more likely to be sampled
        #todo 
        samples = 
        for i, count in enumerate(samples):
            for _ in range(count):
                #get new poses and weights 
                
        #use the new poses and weights
        self.particles.poses = 
        self.particles.weights = 


    def get_estimated_position(self):
        """""
        retrieves the drone's estimated position
        """""
        weights_sum = np.sum(self.particles.weights)
        x = 0.0
        y = 0
        z = 0
        yaw = 0

        normal_weights = self.particles.weights / float(weights_sum)
        for i, prob in enumerate(normal_weights):
            #multiply prob with each pose 
            x += 
            y += 
            z += 
            yaw += 
        #return the particle with all poses x,y,z,yaw
        #also return a np.array with the sum of weights divided by the particles weights size
        return #particle() , np.array())

    def initialize_particles(self, num_particles, kp, des):
        """
        find most possible slocation to start
        :param num_particles: number of particles we are using
        :param kp: the keyPoints of the first captured image
        :param des: the descriptions of the first captured image
        """
        self.key_kp, self.key_des = None, None
        weights_sum = 0.0
        weights = []
        poses = []
        new_poses = []

        # go through every grid, trying to find matched features
        for x in range(MAP_GRID_SIZE_X):
            for y in range(MAP_GRID_SIZE_Y):
                #todo
                p, w = self.compute_location()
                if p is not None:
                    #append all x,y,z and yaw poses
                    poses.append()
                    #append weight with w
                
                    # add w to weights_sum
                    

        # cannot find a match
        if len(poses) == 0:
            print "Random Initialization"
            for x in range(MAP_GRID_SIZE_X):
                for y in range(MAP_GRID_SIZE_Y):
                    #use pose.append 

                    #in pose.append you will need  to add x,y,z  and a random sample

                    #divide cell x by 2.0 and add to the outcome of x * cell x
                    # divide the outcome of the sum by _meter to pixel
                    
                    #divide cell y by 2.0 and add to the outcome of y * cell y
                    # divide the outcome of the sum by _meter to pixel

                    #use z 

                    #using a random sample 


                    poses.append(#todo)


                    weights_sum += 1.0  # uniform sample
                    weights.append(1.0)

        # sample particles based on the number of matched features
        weights =  # normalize
        samples =   # sample
        for i, count in enumerate(samples):
            for _ in range(count):
                #append new poses by i

        self.particles = #particle set with new poses )
        return self.get_estimated_position()

    def compute_location(self, kp1, des1, kp2, des2):
        """
        compute the global location of center of current image
        :param kp1: captured keyPoints
        :param des1: captured descriptions
        :param kp2: map keyPoints
        :param des2: map descriptions
        :return: global pose
        """

        good = []
        pose = None

        if des1 is not None and des2 is not None:
            matches = #use a knn matcher for des1 and des2

            for match in matches:
            #if the length of match 0 is less than 1 and 
            # the distance of match 0 is less that the ratio of match mutiplied by the distance of match 1 

                if :
                     #append good with the match at 0

            #if the length of good is more than the minimal math count
            if len(good) > MIN_MATCH_COUNT:
    
                # Traverse through good in terms of M  
                # queryIdx refers to keypoints1 and trainIdx refers to keypoints2,
                #reshape both pts with -1,1,2

                src_pts = 
                dst_pts = 
                #estimate rigit transfrom with the src pts and dst pts
                
                transform = cv2.estimateRigidTransform( , , False)
                
                if transform is not None:
                    transformed_center =  # get global pixel
                    transformed_center =   # map to global pose
                    
                    yaw =  # get global heading

                    # correct the pose if the drone is not level
                    z = 
                    offset_x = 
                    offset_y = 
                    global_offset_x = #cos
                    global_offset_y = #sin
                    pose = 

        return pose, len(good)

    def compute_transform(self, kp1, des1, kp2, des2):
        transform = None

        if des1 is not None and des2 is not None:
            matches = #use knn match on the des1 and 2 with the k of 2

            good = []
            for match in matches:
                #if the length of match is less than 1 and the distance of match 0 is less than the product of match ratio and match 1 distance
                if 
                    
                    #append match 0 to good  


            # queryIdx refers to keypoints1 and trainIdx refers to keypoints2,
            #reshape both pts with -1,1,2
                    
            src_pts =  
            dst_pts = 

            # estimateRigidTransform needs at least three pairs
            if src_pts is not None and dst_pts is not None and len(src_pts) > 3 and len(dst_pts) > 3:
                #use estimate rigid transform on sec and dst pts
                transform = 

        return transform

    def pixel_to_meter(self, px):
        """""
        uses the camera scale to convert pixel measurements into meter
        """""
        return px * self.z / CAMERA_SCALE


def adjust_angle(angle):
    """""
    keeps angle within -pi to pi
    """""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi

    return angle


def create_map(file_name):
    """
    create a feature map, extract features from each bigger cell.
    :param file_name: the image of map
    :return: a list of center of bigger cells (kp and des), each bigger cell is a 3 by 3 grid (9 cells).
    """

    # read image and extract features
    image = 

    max_total_keypoints = 
    # the edgeThreshold and patchSize can be tuned if the gap between cell is too large
    detector = 
    kp = 
    kp, des = 
    # rearrange kp and des into grid
    grid_kp = 
    grid_des = 
    for i in range(len(kp)):
        x = 
        y = 

        grid_kp[x][y].append(kp[i])
        grid_des[x][y].append(des[i])

    # group every 3 by 3 grid, so we can access each center of grid and its 8 neighbors easily
    map_grid_kp = [[[] for _ in range(MAP_GRID_SIZE_Y)] for _ in range(MAP_GRID_SIZE_X)]
    map_grid_des = [[[] for _ in range(MAP_GRID_SIZE_Y)] for _ in range(MAP_GRID_SIZE_X)]
    for i in range(MAP_GRID_SIZE_X):
        for j in range(MAP_GRID_SIZE_Y):
            for k in range(-1, 2):
                for l in range(-1, 2):
                    x = i + k
                    y = j + l
                    if 0 <= x < MAP_GRID_SIZE_X and 0 <= y < MAP_GRID_SIZE_Y:
                        #fill in the correct values for the 2d array
                        map_grid_kp[i][j].extend(grid_kp[][])
                        map_grid_des[i][j].extend(grid_des[][])

            
            map_grid_des[i][j] = np.array(map_grid_des[][])

    return map_grid_kp, map_grid_des


def norm_pdf(x, mu, sigma):
    u = (x - mu) / float(abs(sigma))
    y = (1 / (np.sqrt(2 * np.pi) * abs(sigma))) * np.exp(-u * u / 2.)
    return y


def distance(x1, y1, x2, y2):
    """""
    returns the distance between two points (x1,y1) and (x2, y2)
    """""
    #todo
    return math.sqrt(math.pow() + math.pow())
