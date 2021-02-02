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

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map and occupancy field
        self.map = OccupancyGrid()
        self.occupancy_field = None


        # the number of particles used in the particle filter
        self.num_particles = 1000

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
        # Finds the positions within the occupancy field that are currently empty
        self.occupancy_field =[]
        for i in range(len(data.data)):
            if data.data[i] == 0:
                self.occupancy_field.append(i)
        # self.occupancy_field = OccupancyField(data)

    

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
            
            q = quaternion_from_euler(0,0,get_yaw_from_pose(p))
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
            self.particle_cloud.append(Particle(p,1))

        self.normalize_particles()
        self.publish_particle_cloud()
        # print ("initialization of particles done...")


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        # pass
        sum_weights = 0
        for part in self.particle_cloud:
            sum_weights = sum_weights + part.w
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
        pass
        #for part in self.particle_cloud:
        #    part.w = 




    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
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
        pass
        # based on the particles within the particle cloud, update the robot pose estimate
        
        # TODO


    
    def update_particle_weights_with_measurement_model(self, data):
        pass
        # TODO data.ranges[0-359]
        #range_data = np.array(data.ranges)
        #minval = min(range_data)
        #mindir = np.argmin(range_data)
        #part_range = np.array(data.ranges)
        #for part in self.particle_cloud:
        #    for i in range(0,360)
        #    part_range

        

    def update_particles_with_motion_model(self):
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        d_x = curr_x - old_x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        d_y = curr_y - old_y
        curr_qx = self.odom_pose.pose.orientation.x
        curr_qy = self.odom_pose.pose.orientation.y
        curr_qz = self.odom_pose.pose.orientation.z
        curr_qw = self.odom_pose.pose.orientation.w
        old_qx = self.odom_pose_last_motion_update.pose.orientation.x
        old_qy = self.odom_pose_last_motion_update.pose.orientation.y
        old_qz = self.odom_pose_last_motion_update.pose.orientation.z
        old_qw = self.odom_pose_last_motion_update.pose.orientation.w
        d_qx = curr_qx-old_qx
        d_qy = curr_qx-old_qy
        d_qz = curr_qx-old_qz
        d_qw = curr_qx-old_qw
        for part in self.particle_cloud:
            part.pose.position.x = part.pose.position.x + d_x
            part.pose.position.y = part.pose.position.y + d_y
            part.pose.orientation.x = part.pose.orientation.x + d_qx
            part.pose.orientation.y = part.pose.orientation.x + d_qy
            part.pose.orientation.z = part.pose.orientation.x + d_qz
            part.pose.orientation.w = part.pose.orientation.x + d_qw


if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









