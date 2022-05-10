#!/usr/bin/env python

import rospy
import tf 
import math
import numpy as np
import time
import threading
import os
import sys
import random

from pozyx_simulation.msg import  uwb_data
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped


class uwb_ranging(object):
    def __init__(self):
        self.robot_pose_x =0
        self.robot_pose_y =0
        self.robot_pose_z =0

        self.counter = 0 

        #get uwb anchors position
        self.sensor_pos = []
        self.sensor_pos = self.get_anchors_pos()

        self.MODELSTATE_INDEX = int(rospy.get_param("~modelstate_index"))
        rospy.loginfo("%s is %s", rospy.resolve_name('modelstate_index'), self.MODELSTATE_INDEX)

        #distances are publishing with uwb_data_distance
        self.pub_uwb_distance = rospy.Publisher('uwb_data_distance', uwb_data, queue_size=10)

        #get robot real position => you can change ModelStates.pose[] different robot's
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.subscribe_data, queue_size=10)

        #start the publish uwb data
        self.uwb_simulate(self.sensor_pos)

    def get_anchors_pos(self):
        max_anchor = 100 
        uwb_id = 'uwb_anchor_'
        listener = tf.TransformListener()

        for i in range(max_anchor):
            try:
                time.sleep(0.3)
                (trans,rot) = listener.lookupTransform('/map', uwb_id+str(i), rospy.Time(0))
                self.sensor_pos.append(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                break

        self.sensor_pos = np.dot(self.sensor_pos,1000)


        if self.sensor_pos == []:
            rospy.logwarn("There is not found any anchors. Function is working again.")    
            self.get_anchors_pos()
        else: 
            rospy.loginfo("UWB Anchor List:\nWarning : uint is mm \n" + str(self.sensor_pos))

        return self.sensor_pos


    def calculate_distance(self, uwb_pose):
        #pose comes in gazebo/model_states (real position)
        robot_pose = [self.robot_pose_x,self.robot_pose_y,self.robot_pose_z]

        #describe 2 points
        p1 = np.array(uwb_pose)
        p2 = np.array(robot_pose)

        #difference between robot and uwb distance
        uwb_dist = np.sum((p1-p2)**2, axis=0)

        #add noise 
        uwb_dist = uwb_dist + np.random.normal(0, uwb_dist*0.01,1)  
        return np.sqrt(uwb_dist)

    def uwb_simulate(self, sensor_pos):

        while not rospy.is_shutdown():
            time.sleep(0.1)
            all_distance = [] 
            all_destination_id = []
            
            for i in range(len(sensor_pos)):
                #calculate distance uwb to robot for all anchors 
                dist = self.calculate_distance(sensor_pos[i])
                if dist >= 25000:
                    all_distance.append(np.nan)
                else:
                    all_distance.append(dist)

                # all_distance.append(dist)


            #uwb_anchors_set.launch same order (not important for simulation)
            all_destination_id.append(0x6e31)
            all_destination_id.append(0x6e32)
            all_destination_id.append(0x6e33)
            all_destination_id.append(0x6e34)
            all_destination_id.append(0x6e35)
            all_destination_id.append(0x6e36)
            all_destination_id.append(0x6e37)
            all_destination_id.append(0x6e38)
            all_destination_id.append(0x6e39)
            all_destination_id.append(0x6e40)
            all_destination_id.append(0x6e41)
            all_destination_id.append(0x6e42)
            all_destination_id.append(0x6e43)
            all_destination_id.append(0x6e44)
            all_destination_id.append(0x6e45)
            all_destination_id.append(0x6e46)
            all_destination_id.append(0x6e47)
            all_destination_id.append(0x6e48)
                
            #publish data with ROS             
            self.publish_data(all_destination_id , all_distance)    


    def publish_data(self, all_destination_id, all_distance):
        #uwb message type is a special message so that firstly describe this message 
        uwb_data_cell = uwb_data()
        uwb_data_cell.destination_id=all_destination_id
        uwb_data_cell.stamp = [rospy.Time.now(),rospy.Time.now(),rospy.Time.now()]
        uwb_data_cell.distance = all_distance
        print("UWB Anchor List: " + str(all_destination_id) + "\n\nDistance: " + str(all_distance) + "\n\n")
        self.pub_uwb_distance.publish(uwb_data_cell)


    def subscribe_data(self, ModelStates):
        self.counter = self.counter + 1 

        # gazebo/modelstate topic frequency is 100 hz. We descrese 10 hz with log method 
        if self.counter % 100 ==  0:  
            self.counter = 0 

            # ModelStates.pose[2] = turtlebot3 model real position on modelstates   
            self.robot_pose_x = ModelStates.pose[self.MODELSTATE_INDEX].position.x*1000
            self.robot_pose_y = ModelStates.pose[self.MODELSTATE_INDEX].position.y*1000
            self.robot_pose_z = ModelStates.pose[self.MODELSTATE_INDEX].position.z*1000
        

if __name__ == "__main__":
    rospy.init_node('uwb_simulation', anonymous=True)

    uwb_simulation = uwb_ranging()
    rospy.spin()

