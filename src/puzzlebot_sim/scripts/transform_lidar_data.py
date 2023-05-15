#!/usr/bin/env python  

import rospy  
from sensor_msgs.msg import LaserScan
import numpy as np


class TransformLidar():  
    def __init__(self):  
        self.new_lidar_pub = rospy.Publisher("base_scan", LaserScan, queue_size=1) 
        rospy.Subscriber("scan", LaserScan, self.lidar_cb ) 
        self.lidar = LaserScan()
        self.received_lidar = 0

        rate=rospy.Rate(10)

        while not rospy.is_shutdown(): 
            if self.received_lidar:
                new_lidar = self.transform_lidar(self.lidar)
                self.new_lidar_pub.publish(new_lidar)
                self.received_lidar = 0
            rate.sleep() 

    def transform_lidar(self,original_lidar):
        #original_lidar is the LaseScan we want to tranform
        #Returns new lidar (LaserScan) with the transformed data
        new_lidar = LaserScan()
        new_ranges = [np.inf]*len(original_lidar.ranges) #init the new array of the original lidar lenght
        new_ranges = np.roll(original_lidar.ranges, len(original_lidar.ranges)/2)
        new_lidar = original_lidar
        new_lidar.ranges = new_ranges
        new_lidar.header.frame_id = "base_link"
        return new_lidar
    

    def lidar_cb(self, msg): 
        self.lidar = msg 
        self.received_lidar = 1 
 


############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    # first thing, init a node! 
    rospy.init_node('transform_lidar_data')  

    TransformLidar()