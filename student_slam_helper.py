"""
student_localization_helper

Fill in the methods according to the docstrings in order to implement
localization on your drone. Run student_run_localization to test your
implementation. We expect that you will re-use code from the particle filter and
OpenCV assignments to complete this task! You should not need to define
any other methods to complete this task, though you may do so if you feel like
it will help you.
"""

import math
import numpy as np
import cv2


# ---------- map parameters ----------- #
MAP_PIXEL_WIDTH = 3227  # in pixel
MAP_PIXEL_HEIGHT = 2447
MAP_REAL_WIDTH = 1.4  # in meter
MAP_REAL_HEIGHT = 1.07
# change the above in order to use a different map 

# ----- camera parameters DO NOT EDIT ----- #
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)
CAMERA_SCALE = 290.
METER_TO_PIXEL = (float(MAP_PIXEL_WIDTH) / MAP_REAL_WIDTH + float(MAP_PIXEL_HEIGHT) / MAP_REAL_HEIGHT) / 2.

# ----- map feature parameters DO NOT EDIT ----- #
ORB_GRID_SIZE_X = 4
ORB_GRID_SIZE_Y = 3
MATCH_RATIO = 0.7
MIN_MATCH_COUNT = 10
MAP_GRID_SIZE_X = ORB_GRID_SIZE_X * 3
MAP_GRID_SIZE_Y = ORB_GRID_SIZE_Y * 3
CELL_X = float(MAP_PIXEL_WIDTH) / MAP_GRID_SIZE_X
CELL_Y = float(MAP_PIXEL_HEIGHT) / MAP_GRID_SIZE_Y
PROB_THRESHOLD = 0.001 # set the weights of new particles to this value
MAP_FEATURES = 600


class Particle(object):
    """
    Each particle holds the complete set of poses and weights for each
    of the particles in the filter. Note that z position should be stored by
    the particle but is not estimated by the particle filter. Each item
    in the poses list will be the list [x,y,z,yaw].
    """

    def __init__(self, i, poses, weights):
        self.i = i
        self.poses = poses
        self.weights = weights

    def weight(self): return self.weights[self.i]

    def x(self): return self.poses[self.i, 0]

    def y(self): return self.poses[self.i, 1]

    def z(self): return self.poses[self.i, 2]

    def yaw(self): return self.poses[self.i, 3]

    def __str__(self):
        return str(self.x()) + ' , ' + str(self.y()) + ' weight ' + str(self.weight())

    def __repr__(self):
        return str(self.x()) + ' , ' + str(self.y()) + ' weight ' + str(self.weight())


class ParticleSet(object):
    """
    Stores the set of particles.
    """

    def __init__(self, num_particles, poses):
        """
        Take the set of poses and the number of particles as input and creates 
        the set of particles. You may assign to each particle the weight 
        PROB_THRESHOLD.
        """

        #######################################################################
        # TODO implement this class 
        # 1 Create the list of particle weights, saved as a 'weights' field
        # 2 Create the list self.particles where each particle gets initialized
        #   with with its number, the list of poses, and the list of weights
        # 3 save self.poses and self.num_particles as fields                                        
        #######################################################################


class LocalizationParticleFilter:
    """
    Particle filter for localization. Contains all of the methods and parameters
    necessary to implement the Monte Carlo localization algorithm. You should
    not need to edit anything in the init method below.
    """

    def __init__(self, map_kp, map_des):
        self.map_kp = map_kp
        self.map_des = map_des

        self.particles = None
        self.measure_count = 0

        index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)

        self.previous_time = None

        self.key_kp = None
        self.key_des = None

        self.z = 0.0
        self.angle_x = 0.0
        self.angle_y = 0.0

        self.sigma_x = 0.05
        self.sigma_y = 0.05
        self.sigma_yaw = 0.01

        sigma_vx = 0.01
        sigma_vy = 0.01
        sigma_vz = 0.0
        sigma_yaw = 0.01
        self.covariance_motion = np.array([[sigma_vx ** 2, 0, 0, 0],
                                           [0, sigma_vy ** 2, 0, 0],
                                           [0, 0, sigma_vz ** 2, 0],
                                           [0, 0, 0, sigma_yaw ** 2]])

    def update(self, z, angle_x, angle_y, prev_kp, prev_des, kp, des):
        """
        We implement the MCL algorithm from probabilistic robotics (Table 8.2)
        :param kp: the positions of detected features
        :param des: the descriptors of detected features
        :param prev_kp, prev_des: kp and des from the previous frame
        :param angle_x, angle_y: the current roll and pitch of the drone
        :param z: the current height of the drone (not filtered)
        :return: the drone's position estimate

        Do a motion update on every frame, and a measurement update whenever
        there is a keyframe (the drone has moved a certain distance since the
        last time we determined a keyframe).You must choose a threshold for 
        the distance which must pass between camera frames to trigger a measurement 
        update.

        The kp and des of the previous keyframe should be stored in self.kp
        and self.des. You should use self.compute_transform to find the 
        distance between the previous camera frame and the current one 
        to use as a control input for the motion model. 

        You must then determine when to perform a measurement update by 
        comparing the current image to the last keyframe image. Do this by
        comparing the current kp and des to self.key_kp and self.key_des. Make sure
        you consider what will happen when:
        1. there is no previous keyframe
        2. the camera has moved too far to transform from the previous keyframe
        
        Also consider when it is necessary to resample!
        """
        # update parameters
        self.z = z
        self.angle_x = angle_x
        self.angle_y = angle_y

        #######################################################################
        # TODO: implement this method!
        #
        # Note: if transform is the output of self.compute_transform then:
        #
        # x = -transform[0, 2]
        # y = transform[1, 2]
        # yaw = -np.arctan2(transform[1, 0], transform[0, 0])
        # all of which are measured in pixels
        #
        # be sure to return the estimated position 
        #######################################################################

    def motion_model(self, x, y, yaw):
        """
        Update the x,y, and yaw poses of each particle in the filter using the
        distance between the previous camera frame and this one as a motion
        estimate. Be sure to add Gaussian noise to the x,y,yaw distance values
        and use different noise for each particle!

        Hint: use the predefined self.covariance_motion for the covariance as 
        an input to numpy.random.multivariate_normal to sample from the normal 
        distribution. You will likely need to look at the documentation of this 
        function.

        :param x: the estimated distance in x between the previous frame and this
                  one, in meters
        :param y: the estimated distance in y between the previous frame and this
                  one, in meters
        :param yaw: the estimated yaw difference between the previous frame and
                    this one, in radians
        :return: nothing
        """

        #######################################################################
        # TODO implement this method
        #######################################################################

    def measurement_model(self, kp, des):
        """
        The measurement model sets the weight for each particle in the filter.
        The weight should reflect the error between the pose of a particle and
        the "true" pose of the robot. We will obtain the true pose by computing 
        the location of the drone's current image in the larger map.

        To implement this method, you should follow the "landmark model known
        correspondence" algorithm in the textbook online.

        We have provided a bit of code which will get the kp and
        des from the 8 map grids immediately surrounding the pose of each
        particle. Your job is to compute the probability that each particle's pose is
        the correct pose of the drone and set the weight of the particle to that
        probability.

        We have provided a function called "norm pdf" which allows you to obtain
        the probability of a particular sample given a mean and variance. See the
        docstring on the method in order to use it. You should use this method
        to determine the probability of each particle's error. Specifically, use
        norm_pdf with a mean of zero, which will give you the probability of drawing
        a sample from the normal distribution with a mean of zero, ie numbers far 
        from zero have lower probability of being sampled
        from this distribution. The sample you should consider is the error
        between the pose of each particle and the true pose of the drone.

        :param kp: the currently observed keypoints
        :param des: the currently observed descriptors
        :return: nothing
        """
        for i in range(self.particles.num_particles):
            position = self.particles.poses[i]

            # get grid x and y
            grid_x = int(position[0] * METER_TO_PIXEL / CELL_X)
            grid_x = max(min(grid_x, MAP_GRID_SIZE_X - 1), 0)
            grid_y = int(position[1] * METER_TO_PIXEL / CELL_Y)
            grid_y = max(min(grid_y, MAP_GRID_SIZE_Y - 1), 0)
            # get 8 map cells around pose
            sub_map_kp = self.map_kp[grid_x][grid_y]
            sub_map_des = self.map_des[grid_x][grid_y]

            # measure current global position
            pose, num = self.compute_location(kp, des, sub_map_kp, sub_map_des)

            #######################################################################
            # TODO implement the pseucode below!
            #
            # Note: use self.sigma_x/y/yaw to use as the variance for norm_pdf and
            # for np.random.normal
            #
            # 1 add some noise to the global pose (pose)
            # 2 find the errors between the global pose (pose) and the pose of 
            #   each particle (position)
            # 3 call adjust_angle on the difference of yaw to keep it within
            #   -pi/2 and pi/2
            # 4 find the probabilities of the errors using norm_pdf (see docstring above)
            # 5 set the weight of the particle to the max of PROB_THRESHOLD and the
            #   product of the probabilities for the error in x, y, and yaw
            #######################################################################

    def resample_particles(self):
        """
        Samples a new particle set, reproducing each particle with a probability
        proportional to its weight. 
        """

        weights_sum = np.sum(self.particles.weights)
        new_poses = []
        new_weights = []

        normal_weights = self.particles.weights / float(weights_sum)  # normalize
        # samples sums to num_particles with same length as normal_weights, 
        # positions with higher weights are more likely to be sampled
        samples = np.random.multinomial(self.particles.num_particles, normal_weights)
        for i, count in enumerate(samples):
            for _ in range(count):
                new_poses.append(self.particles.poses[i])
                new_weights.append(self.particles.weights[i])

        self.particles.poses = np.array(new_poses)
        self.particles.weights = np.array(new_weights)

    def get_estimated_position(self):
        """
        Return the drone's estimated position using the expectation of the
        belief distribution, which is the weighted sum of the poses of all the
        particles in the filter, weighted by the weight of the particle. Note
        that the weights must be normalized to 1 for this to work.

        eg: the estimated x position of the drone is the sum over all of the
              particles of the product of the normalized weight of the
              particle and the particle's x pose
        """

        #######################################################################
        # TODO implement this method

        # 1. compute the estimated values x, y, z, yaw 
        # 2. compute the average weight value: avg_weight
        #######################################################################

        return Particle(0, np.array([[x, y, z, yaw]]), np.array([avg_weight]))

    def initialize_particles(self, num_particles, kp, des):
        """
        find most possible location to start
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
                p, w = self.compute_location(kp, des, self.map_kp[x][y], self.map_des[x][y])
                if p is not None:
                    poses.append([p[0], p[1], p[2], p[3]])
                    weights.append(w)
                    weights_sum += w

        # cannot find a match
        if len(poses) == 0:
            print "Random Initialization"
            for x in range(MAP_GRID_SIZE_X):
                for y in range(MAP_GRID_SIZE_Y):
                    poses.append([(x * CELL_X + CELL_X / 2.0) / METER_TO_PIXEL,
                                  (y * CELL_Y + CELL_Y / 2.0) / METER_TO_PIXEL,
                                  self.z,
                                  np.random.random_sample() * 2 * np.pi - np.pi])
                    weights_sum += 1.0  # uniform sample
                    weights.append(1.0)

        # sample particles based on the number of matched features
        weights = np.array(weights) / weights_sum  # normalize
        samples = np.random.multinomial(num_particles, weights)  # sample
        for i, count in enumerate(samples):
            for _ in range(count):
                new_poses.append(poses[i])

        self.particles = ParticleSet(num_particles, np.array(new_poses))
        return self.get_estimated_position()
        
    def compute_transform(self, kp1, des1, kp2, des2):
        """
        Returns the pixel transformation matrix (output of estimateRigidTransform)
        from kp1 to kp2. This is used to find the distance between camera frames.
        This should be entirely code from the OpenCV assignment!

        :param kp1: the first list of keypoints
        :param des1: the first list of descriptors
        :param kp2: the second list of keypoints
        :param des2: the second list of descriptors
        :return: the transformation matrix from kp1 to kp2
        """

        #######################################################################
        # TODO fill in this method with your code from the OpenCV task
        #######################################################################


    def compute_location(self, kp1, des1, kp2, des2):
        """
        Compute the global location of the center of the current image in the
        map. You do not need to edit this method.

        :param kp1: captured keyPoints
        :param des1: captured descriptions
        :param kp2: map keyPoints
        :param des2: map descriptions
        :return: global pose
        """

        good = []
        pose = None

        if des1 is not None and des2 is not None:
            matches = self.matcher.knnMatch(des1, des2, k=2)

            for match in matches:
                if len(match) > 1 and match[0].distance < MATCH_RATIO * match[1].distance:
                    good.append(match[0])

            if len(good) > MIN_MATCH_COUNT:
                src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
                transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)
                if transform is not None:
                    transformed_center = cv2.transform(CAMERA_CENTER, transform)  # get global pixel
                    transformed_center = [transformed_center[0][0][0] / METER_TO_PIXEL,  # map to global pose
                                          (MAP_PIXEL_HEIGHT - 1 - transformed_center[0][0][1]) / METER_TO_PIXEL]
                    yaw = np.arctan2(transform[1, 0], transform[0, 0])  # get global heading

                    # correct the pose if the drone is not level
                    z = math.sqrt(self.z ** 2 / (1 + math.tan(self.angle_x) ** 2 + math.tan(self.angle_y) ** 2))
                    offset_x = np.tan(self.angle_x) * z
                    offset_y = np.tan(self.angle_y) * z
                    global_offset_x = math.cos(yaw) * offset_x + math.sin(yaw) * offset_y
                    global_offset_y = math.sin(yaw) * offset_x + math.cos(yaw) * offset_y
                    pose = [transformed_center[0] + global_offset_x, transformed_center[1] + global_offset_y, z, yaw]

        return pose, len(good)


    def pixel_to_meter(self, px):
        """
        Uses the CAMERA_SCALE to return the pixel measurement converted into
        meters. Note that this requires knowledge of the height of the drone.
        You should use your code from the OpenCV assignment to implement this
        method.

        :param px: the pixel measurement to convert to meters
        :return: the meter equivalent of the pixel distance
        """

        #######################################################################
        # TODO fill in this method with your code from the OpenCV task
        #######################################################################

def adjust_angle(angle):
    """
    Returns angle, ensuring that it stays within the range -pi to pi

    :param angle: the angle to keep within -pi and pi
    :return: angle adjusted to stay within the range -pi to pi
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi

    return angle


def create_map(file_name):
    """
    Creates a feature map by extracting features from the map image and sorting
    them intro grids. You do not need to edit this method.

    :param file_name: the image of the map
    :return: the map_grid_des and map_grid_kp, two dimensional lists of the
             kp and des from the map which are grouped into cells
    """

    # read image and extract features
    image = cv2.imread(file_name)
    # the edgeThreshold and patchSize can be tuned if the gap between cell is too large
    detector = cv2.ORB(nfeatures=MAP_FEATURES, scoreType=cv2.ORB_FAST_SCORE)
    max_total_keypoints = 500 * ORB_GRID_SIZE_X * ORB_GRID_SIZE_Y
    detector_grid = cv2.GridAdaptedFeatureDetector(detector, maxTotalKeypoints=max_total_keypoints,
                                                   gridCols=ORB_GRID_SIZE_X, gridRows=ORB_GRID_SIZE_Y)
    kp = detector_grid.detect(image, None)
    kp, des = detector.compute(image, kp)

    # rearrange kp and des into grid
    grid_kp = [[[] for _ in range(MAP_GRID_SIZE_Y)] for _ in range(MAP_GRID_SIZE_X)]
    grid_des = [[[] for _ in range(MAP_GRID_SIZE_Y)] for _ in range(MAP_GRID_SIZE_X)]
    for i in range(len(kp)):
        x = int(kp[i].pt[0] / CELL_X)
        y = MAP_GRID_SIZE_Y - 1 - int(kp[i].pt[1] / CELL_Y)
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
                        map_grid_kp[i][j].extend(grid_kp[x][y])
                        map_grid_des[i][j].extend(grid_des[x][y])
            map_grid_des[i][j] = np.array(map_grid_des[i][j])

    return map_grid_kp, map_grid_des


def norm_pdf(x, mu, sigma):
    """
    Finds the probability of obtaining the sample x from a normal distribution
    with mean mu and variance sigma. You do not need to edit this method.

    :param x: the sample to find the probability of
    :param mu: the mean of the normal distribution
    :param sigma: the variance of the normal distribution
    :return: the probability of obtaining the sample x from a Gaussian with
             mean mu and variance sigma
    """

    u = (x - mu) / float(abs(sigma))
    y = (1 / (np.sqrt(2 * np.pi) * abs(sigma))) * np.exp(-u * u / 2.)
    return y
