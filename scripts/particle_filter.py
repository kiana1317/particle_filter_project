#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample 
import math
from copy import deepcopy

from random import randint, random, uniform



def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(choices, probabilities, n):
    """ Return a random sample of n elements from the set choices with the specified probabilities
        choices: the values to sample from represented as a list
        probabilities: the probability of selecting each element in choices represented as a list
        n: the number of samples
    """
    values = np.array(range(len(choices)))
    probs = np.array(probabilities)
    bins = np.add.accumulate(probs)
    inds = values[np.digitize(random_sample(n), bins)]
    samples = []
    for i in inds:
        samples.append(deepcopy(choices[int(i)]))
    return samples


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # Attempt 1 to fix timing
        # wait = True
        # while wait:
        #     last_ros_time_ = rospy.get_time()
        #     if (last_ros_time_ > 0):
        #          wait = False
        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map and occupancy field
        self.map = OccupancyGrid()
        self.occupancy_field = None


        # the number of particles used in the particle filter
        self.num_particles = 5000  # more than about 1000 particles causes the wierd "no such transformation" error

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):

        self.map = data
        # self.occupancy_field = OccupancyField(data)
        # Finds the positions within the occupancy field that are currently empty
        self.occupancy_field =[]
        for i in range(len(data.data)):
            if data.data[i] == 0:
                self.occupancy_field.append(i)

    

    def initialize_particle_cloud(self):
        # Wait until the occupancy field is populated
        while not self.occupancy_field:
            pass

        choices = self.occupancy_field
        probability = 1.0 / len(choices)
        probabilities = [probability] * len(choices)
        n = self.num_particles
        particles_sample = draw_random_sample(choices, probabilities, n)

        for i in particles_sample:
            p = Pose()
            # translate the row grid to coordinates
            width = self.map.info.width
            origin_x = self.map.info.origin.position.x
            origin_y = self.map.info.origin.position.y
            p.position.y = origin_y + (i / width * self.map.info.resolution) 
            p.position.x = origin_x + (i % width * self.map.info.resolution)
            angle = uniform(0,2*math.pi)        # set a random yaw angle
            q = quaternion_from_euler(0,0,angle) # quaternion for orientation
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
            self.particle_cloud.append(Particle(p,1))

        self.normalize_particles()
        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0 by dividing by sum over all particle weights
        sum_weights = 0.0
        for part in self.particle_cloud:
            sum_weights += part.w
        for part in self.particle_cloud:
            part.w = part.w / sum_weights

    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)


    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)

    def resample_particles(self):
        # Simpler Implementation
        probabilities = []
        for part in self.particle_cloud:
            probabilities.append(part.w)
        n = self.num_particles
        self.particle_cloud = draw_random_sample(self.particle_cloud, probabilities, n)
        
        #############
        # minthresh = 10/self.num_particles # this weight threshold keeps about 90% of the particles 
        
        # # Initialize Probability Vector
        # probs =[]
        # for part in self.particle_cloud:
        #     probs.append(part.w)

        # # make bins for the cumulative distribution from the weights
        # bins = np.add.accumulate(probs)
        # for part in self.particle_cloud:
        #     if (part.w < minthresh):      # if the particle weight is below the threshold we resample it
        #         # sample from the distribution
        #         idx = self.particle_cloud[np.digitize(random(),bins)]

        #         # update the particle position to the sampled particle, and add a normal noise offset to x and y position
        #         part.pose.position.x = idx.pose.position.x + np.random.normal()*self.map.info.resolution*2
        #         part.pose.position.y = idx.pose.position.y + np.random.normal()*self.map.info.resolution*2
        #         # update the yaw angle to the sampled particle and add a small random angle offset
        #         yaw = get_yaw_from_pose(idx.pose)+(random()-.5)*math.pi/8
        #         q = quaternion_from_euler(0,0,yaw)
        #         q = q/math.sqrt(q[0]**2+q[1]**2+q[2]**2+q[3]**2) # normalize the quaternion since ROS complained 
        #         part.pose.orientation.x = q[0]
        #         part.pose.orientation.y = q[1]
        #         part.pose.orientation.z = q[2]
        #         part.pose.orientation.w = q[3]

    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        try:
            self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
            print("Transform work!")
        except:
            print("Waiting for transform...")
            return

        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        #pass
        # based on the particles within the particle cloud, update the robot pose estimate
        newpos = Pose()
        for part in self.particle_cloud: # loop over the particles and average the poses of all particles
            newpos.position.x += (part.pose.position.x/self.num_particles)
            newpos.position.y += (part.pose.position.y/self.num_particles)
            newpos.orientation.x += (part.pose.orientation.x/self.num_particles)
            newpos.orientation.y += (part.pose.orientation.y/self.num_particles)
            newpos.orientation.z += (part.pose.orientation.z/self.num_particles)
            newpos.orientation.w += (part.pose.orientation.w/self.num_particles)
            # Not sure the purpose for normalization -Kiana
            # norm = math.sqrt(newpos.orientation.x**2 + newpos.orientation.y**2+ newpos.orientation.z**2
            #             +newpos.orientation.w**2)
            # newpos.orientation.x = newpos.orientation.x / norm # normalize the quaternions
            # newpos.orientation.y = newpos.orientation.y / norm # since ROS complained about an unnormalized one
            # newpos.orientation.z = newpos.orientation.z / norm
            # newpos.orientation.w = newpos.orientation.w / norm
        self.robot_estimate= newpos


    def update_particle_weights_with_measurement_model(self, data):
        thresh = .1   # used to find the walls with the ray tracing method
        range_data = np.array(data.ranges) # scanner range data
        max_r = np.amax(range_data,where=~np.isinf(range_data),initial=-1,axis=0)
        # this is the largest non-infinite range from the scanner
        forval = min(range_data[0],max_r) # straight ahead scanner value
        lval = min(range_data[90],max_r)  # left scanner value
        bval = min(range_data[180],max_r) # behind scanner value
        rval = min(range_data[270],max_r) # right scanner value
        min_x = self.map.info.origin.position.x
        min_y = self.map.info.origin.position.y
        max_x = min_x + self.map.info.resolution*(self.map.info.width-1)
        max_y = min_y + self.map.info.resolution*(self.map.info.height-1)
        for part in self.particle_cloud:
            px = part.pose.position.x     # coordinates of particle
            py = part.pose.position.y
            col = int((px - min_x)/self.map.info.resolution)
            row = int((py - min_y)/self.map.info.resolution) 
            index = row*self.map.info.width + col
            occ = self.map.data[index]    # particle in the house?
            zeroout = 0                  
            if (occ > thresh or occ < 0): # if outside or in wall
                zeroout = 1               # then set weight to zero   
            yaw = get_yaw_from_pose(part.pose) # particle yaw
            dr = self.map.info.resolution/2    # small step for ray tracing
            err = 0
            for i in [0,90,180,270]:      # look front, left, back, and right
                hit = 0
                rr = 0
                if range_data[i] > 5: # this is the case when it's infinite
                    erri = 0  
                else:    
                    while (hit == 0):# trace a ray until we hit a wall 
                        rr = rr + dr  # move ray forward
                        sx = px + math.cos(yaw+i*math.pi/180)*rr 
                        sy = py + math.sin(yaw+i*math.pi/180)*rr 
                        col = int((sx - min_x)/self.map.info.resolution)
                        row = int((sy - min_y)/self.map.info.resolution) 
                        index = row*self.map.info.width + col
                        occ = self.map.data[index]  # check to see if hit a wall yet, or outside
                        if (occ > thresh or occ < 0 or sx > max_x or sy > max_y
                            or sx < min_x or sy < min_y or rr>max_x): # if occupied then we hit the wall
                            hit = 1
                            # if the laser has a valid value in this direction, calculate the error
                            # if the laser is infinite there, then we'll ignore this direction
                    erri = (range_data[i]-rr) 
                err = err + erri**2 # add up square errors
            if zeroout == 1: 
                part.w = 0  # if we planned to zero this particle out, set weight to zero
            else:
                part.w = 1/err # make the weight 1/square error
        self.normalize_particles() # renormalize the particles

    def update_particles_with_motion_model(self):
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        d_x = curr_x - old_x

        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        d_y = curr_y - old_y

        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        d_yaw = curr_yaw - old_yaw
        for part in self.particle_cloud:
            # move the particle x and y coordinates, and add a small random amount in each direction
            part.pose.position.x += (d_x + np.random.normal()*self.map.info.resolution*2)
            part.pose.position.y += (d_y + np.random.normal()*self.map.info.resolution*2)
            # update the yaw, and then add a small uniform error to the yaw 
            q = quaternion_from_euler(0,0,get_yaw_from_pose(part.pose)+d_yaw+uniform(-1,1)*math.pi/16)
            # q = q/math.sqrt(q[0]**2+q[1]**2+q[2]**2+q[3]**2) # normalize the quaternion - Not sure why this is needed
            part.pose.orientation.x = q[0]
            part.pose.orientation.y = q[1]
            part.pose.orientation.z = q[2]
            part.pose.orientation.w = q[3]

if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









