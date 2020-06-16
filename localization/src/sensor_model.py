import numpy as np
import time
from scan_simulator_2d import PyScanSimulator2D

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

class SensorModel:


    def __init__(self):

        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")
        self.INV_SQUASH_FACTOR = 1.0 / float(rospy.get_param("~squash_factor"))

        ####################################
        # Precompute the sensor model here
        # (You should probably write a
        #  function for this)
        self.MAX_RANGE_PX = 200
        self.sensor_model_table = None
        self.particle_scans = None
        self.precompute_sensor_model()
        ####################################

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
                self.num_beams_per_particle,
                self.scan_field_of_view,
                0, # This is not the simulator, don't add noise
                0.01, # This is used as an epsilon
                self.scan_theta_discretization) 

        # Subscribe to the map
        self.map_set = False
        self.map = None
        self.permissible_region = None 
        self.map_info = None
        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)

    def precompute_sensor_model(self):
        '''
        Generate and store a table which represents the sensor model. For each discrete computed
        range value, this provides the probability of measuring any (discrete) range.
        This table is indexed by the sensor model at runtime by discretizing the measurements
        and computed ranges from RangeLibc.
        '''
        print("Precomputing sensor model")
        # sensor model constants
        alpha_hit = 0.74
        alpha_short = 0.07
        alpha_max = 0.07
        alpha_rand = 0.12
        sigma_hit = 8.0 #convert to px space

        table_width = int(self.MAX_RANGE_PX) + 1
        self.sensor_model_table = np.zeros((table_width,table_width))

        hit_array = np.zeros(table_width)
        t = time.time()
        # d is the computed range from RangeLibc
        for d in range(table_width):
            norm = 0.0
            sum_unkown = 0.0
            hit_sum = 0.0
            #short_sum = 0.0
            #max_sum = 0.0
            #rand_sum = 0.0
            # r is the observed range from the lidar unit
            for r in range(table_width):
                prob = 0.0
                z = float(r-d)
                # reflects from the intended object
                hit_val = np.exp(-(z*z)/(2.0*sigma_hit*sigma_hit)) / (sigma_hit * np.sqrt(2.0*np.pi))
                hit_sum += hit_val
                hit_array[r] = hit_val
                #prob += alpha_hit * np.exp(-(z*z)/(2.0*sigma_hit*sigma_hit)) / (sigma_hit * np.sqrt(2.0*np.pi))

                # observed range is less than the predicted range - short reading
                if r < d:
                    prob += 2.0 * alpha_short * (d - r) / float(d*d)
                    #short_sum += 2.0 * (d - r) / float(d*d)

                # erroneous max range measurement
                if int(r) == int(self.MAX_RANGE_PX):
                    prob += alpha_max
                    #max_sum += alpha_max

                # random measurement
                if r < int(self.MAX_RANGE_PX):
                    prob += alpha_rand * 1.0/float(self.MAX_RANGE_PX)
                    #rand_sum += 1.0/float(self.MAX_RANGE_PX)

                norm += prob
                self.sensor_model_table[int(r),int(d)] = prob
            # normalize
            self.sensor_model_table[:,int(d)] += alpha_hit * hit_array / hit_sum
            norm += alpha_hit
            self.sensor_model_table[:,int(d)] /= norm

    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar.

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        if not self.map_set:
            return

        ####################################
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle 

        
        self.particle_scans = self.scan_sim.scan(particles)

        observation /= float(self.map_info.resolution)
        self.particle_scans /= float(self.map_info.resolution)

        observation[observation > self.MAX_RANGE_PX] = self.MAX_RANGE_PX
        self.particle_scans[self.particle_scans > self.MAX_RANGE_PX] = self.MAX_RANGE_PX
        observation[observation < 0 ] = 0
        self.particle_scans[self.particle_scans < 0] = 0 

        intobs = np.rint(observation).astype(np.uint16)
        intscn = np.rint(self.particle_scans).astype(np.uint16)

        prob = np.prod(self.sensor_model_table[intobs, intscn], axis=1)
        np.power(prob, self.INV_SQUASH_FACTOR, prob)
        return prob

        ####################################

    def get_scans(self, particles):
        return self.scan_sim.scan(particles)
 
    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double)/100.
        self.map = np.clip(self.map, 0, 1)
        self.map_info = map_msg.info

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])
        print("map origin", origin)

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
                self.map,
                map_msg.info.height,
                map_msg.info.width,
                map_msg.info.resolution,
                origin,
                0.5) # Consider anything < 0.5 to be free

        # 0: permissible, -1: unmapped, 100: blocked
        array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        # 0: not permissible, 1: permissible
        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255==0] = 1 
        # Make the map set
        self.map_set = True

        print("Map initialized")
