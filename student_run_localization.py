"""""
picam_localization_distance

implements Monte-Carlo Localization using the pi-camera
"""""

import numpy as np
import cv2
from pidrone_pkg.msg import Mode
from sensor_msgs.msg import Image, Range, CameraInfo
from std_msgs.msg import Empty
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from pidrone_pkg.msg import State
from localization_helper import LocalizationParticleFilter, create_map, PROB_THRESHOLD
import os

# ---------- map parameters ----------- #
MAP_PIXEL_WIDTH = 3227  # in pixel
MAP_PIXEL_HEIGHT = 2447
MAP_REAL_WIDTH = 1.4  # in meter
MAP_REAL_HEIGHT = 1.07
# ------------------------------------- #

# ---------- camera parameters ----------- #
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
METER_TO_PIXEL = (float(MAP_PIXEL_WIDTH) / MAP_REAL_WIDTH + float(MAP_PIXEL_HEIGHT) / MAP_REAL_HEIGHT) / 2.
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)
# ---------------------------------------- #

# ---------- localization parameters ----------- #
MAX_BAD_COUNT = -10
NUM_PARTICLE = 30
NUM_FEATURES = 200
# ---------------------------------------------- #


class AnalyzePhase:

    def __init__(self):
        self.bridge = CvBridge()
        self.br = tf.TransformBroadcaster()

        self.posepub = 
        self.first_image_pub = 
        #opencv orb  create
        self.detector = 
        #use map.jpg to create a map
        map_grid_kp, map_grid_des = 

        #use map grid kp and des inside the localizationparticlefilter
        self.estimator = 

        # [x, y, z, yaw]
        self.pos = [0, 0, 0, 0]
        self.posemsg = PoseStamped()

        self.angle_x = 0.0
        self.angle_y = 0.0
        # localization does not estimate z
        self.z = 0.0

        self.first_locate = True
        self.locate_position = False

        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.prev_time = None
        self.prev_rostime = None

        self.map_counter = 0
        self.max_map_counter = 0

        # constant
        self.alpha_yaw = 0.1  # perceived yaw smoothing alpha
        self.hybrid_alpha = 0.3  # blend position with first frame and int

    #todo image callback
    def image_callback(self, data):
       
        curr_img = 
        curr_rostime = =
        self.posemsg.header.stamp = 
        curr_time = 

        #todo
        # start MCL localization
        if self.locate_position:
            curr_kp, curr_des = 

            if curr_kp is not None and curr_kp is not None:
                # generate particles for the first time
                if self.first_locate:
                    particle = 
                    self.first_locate = 
                    self.pos = 

                    self.posemsg.pose.position.x = 
                    self.posemsg.pose.position.y = 
                    self.posemsg.pose.position.z = 
                    x, y, z, w = 

                    self.posemsg.pose.orientation.x = 
                    self.posemsg.pose.orientation.y = 
                    self.posemsg.pose.orientation.z = 
                    self.posemsg.pose.orientation.w = 

                    print 'first', particle
                else:
                    particle = 

                    # update position
                    self.pos = 


                    self.posemsg.pose.position.x = 
                    self.posemsg.pose.position.y =
                    self.posemsg.pose.position.z = 
                    x, y, z, w = 

                    self.posemsg.pose.orientation.x = 
                    self.posemsg.pose.orientation.y = 
                    self.posemsg.pose.orientation.z = 
                    self.posemsg.pose.orientation.w = 
                    self.posepub.publish(self.posemsg)

                    print '--pose', self.pos[0], self.pos[1], self.pos[3]

                    # if all particles are not good estimations
                    #todo
                    if is_almost_equal(particle.weight(), PROB_THRESHOLD):
                        #subtract map counter by 1 
                        self.map_counter = 
                    elif self.map_counter <= 0:
                        #map counter equals 1 
                        self.map_counter = 1
                    else:
                        #use the min() function 
                        #min()has 2 arguments
                        #add to map counter and use -MAX_BAD_COUNT
                        self.map_counter = 

                    # if it's been a while without a significant average weight
                    if self.map_counter < MAX_BAD_COUNT:
                        self.first_locate = True
                        self.map_counter = 0
                        print 'Restart localization'

                    print 'count', self.map_counter
            else:
                print "CANNOT FIND ANY FEATURES !!!!!"

            self.prev_kp = curr_kp
            self.prev_des = curr_des

        self.prev_img = curr_img
        self.prev_time = curr_time
        self.prev_rostime = curr_rostime
        self.br.sendTransform((self.pos[0], self.pos[1], self.z),
                              tf.transformations.quaternion_from_euler(0, 0, self.pos[3]),
                              rospy.Time.now(),
                              "base",
                              "world")

    def state_callback(self, data):
        """ update z, angle x, and angle y data when /pidrone/state is published to """
       #update with converiance
        self.z = 
        self.angle_x = 
        self.angle_y = 

    def reset_callback(self, data):
        """start localization when '/pidrone/reset_transform' is published to (press 'r')"""
        print "Start localization"
        #SET TO TRUE
         
        #SET TO ZERO
         


def is_almost_equal(x, y):
    epsilon = 1*10**(-8)
    return abs(x-y) <= epsilon


def main():
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    
    phase_analyzer = AnalyzePhase()
s
    phase_analyzer = AnalyzePhase()
    #subcriber to reset_transform , raspicam_node and pidrone_state 
    #with the correct callback
    rospy.Subscriber("")
    rospy.Subscriber("")
    rospy.Subscriber("")

    print"Start")

    rospy.spin()


if __name__ == '__main__':
    main()
