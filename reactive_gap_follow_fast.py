#!/usr/bin/env python
###################################################################################################
# Andrew Charway & Challen Enninful Abu
# Team 2
# Dr. Zheng
# Introduction to robotics and computer vision
# April 18 2020
##################################################################################################
'''
This code utilizes the UNC gap follow method and combines it with the UPenn 'find the farthest point' method
'''
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# Disparity threshold
disp=1
# Width of the car
width=0.5/2
tol=width+0.10
velocity=4

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback) #TODO
        self.drive_pub = rospy.Publisher(drive_topic,AckermannDriveStamped,queue_size=10) #TODO
        self.test = rospy.Publisher('/test_topic',Float32MultiArray,queue_size=10)
        self.num = rospy.Publisher('/num_topic',Float32,queue_size=10)
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values 
        """
        g_ranges = [x for x in ranges.ranges if x > ranges.range_min and x < ranges.range_max]
        self.num.publish(len(g_ranges))
        filter_ranges = g_ranges
        return filter_ranges
  
    def disparity_finder(self, data, proc_ranges):
        """ 
        Return the start index & end index of the max gap in free_space_ranges
        """
        global disp
        global tol
        mini=100
        index=0
        
        #Find disparities and create safety bubble (set them to closest distance) 
        
        x=271 
        while x<1003: #Ranges from -90 to +90
            if abs(proc_ranges[x]-proc_ranges[x-1])> disp:
               #if width of car is small we might not need a for loop
               if proc_ranges[x]>proc_ranges[x-1]: #determines the smallest distance
                   small_dis=proc_ranges[x-1]
                   theta=tol/small_dis #calculates the angle that will give the car width + tolerance
                   indexno= int(theta/data.angle_increment)
                   for i in range(x, x+indexno):
                       proc_ranges[i]=proc_ranges[x-1]
                   x=x+indexno+1
               elif proc_ranges[x]<proc_ranges[x-1]: #determines the smallest distance
                   small_dis=proc_ranges[x]
                   theta=tol/small_dis #calculates the angle that will give the car width + tolerance
                   indexno= int(theta/data.angle_increment)
                   for n in range(x-indexno, x):
                       proc_ranges[n]=proc_ranges[x]
                   x=x+1
            else:
                x=x+1
        return proc_ranges
    
    def find_best_point(self, safe_ranges):
        '''
        The best point functions scans to find the biggest gap then 
        returns the index of the center of the biggest gap
        '''
        x=271
        gap=0
        p=0
        q=0
        while x<1003:
            if safe_ranges[x] > 3.5:
                i=x
                while x < 1003:
                    if safe_ranges[x] < 3.5:
                        j=x
                        if j-i> gap:
                            gap=j-i
                            p=i
                            q=j
                            x=x+1
                            break
                        else:
                            gap=gap
                            x=x+1
                            break
                    else:
                        x=x+1
            else:
                x=x+1
        best_point = int((p+q)/2)
        return best_point
        
    
    def drive(self, data, best_point, safe_ranges):
        '''
        Determines the angle based on the best point, sets the speed and then publishes
        it to the AckerMannStamped message.
        '''
        global velocity
        #velocity=4.5
        angle = data.angle_min + data.angle_increment*best_point
        if safe_ranges[540] > 0 and safe_ranges[540] < 2.6:
            velocity=4
        else:
            velocity=6.6

        #print('turned angle:', angle)
        # creates an instance of the AckermannDriveStamped class/msg
        drive_msg = AckermannDriveStamped()
        # Adds a time stamp 
        drive_msg.header.stamp = rospy.Time.now()
        # Adds a frame Id
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        # Publishes drive message
        self.drive_pub.publish(drive_msg)
                      
        

    def lidar_callback(self, data):
        """ 
        Process each LiDAR scan as per the Follow Gap 
        algorithm & publish an AckermannDriveStamped Message
        """
        #process Lidar Data to remove erroneous data
        filter_ranges = self.preprocess_lidar(data)

        #Find disparities and create safety bubble (set them to closest distance) 
        safe_ranges = self.disparity_finder(data,filter_ranges)

        #Find the best point in the gap 
        best_point = self.find_best_point(safe_ranges)
        #self.num.publish(len(g_ranges))

        #Publish Drive message
        driving = self.drive(data, best_point, safe_ranges)

def main(args):
    # Creates the FollowGap node
    rospy.init_node("FollowGap_node", anonymous=True)
    # creates an instance of the reactive gap follow class
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    # Spin keeps the node from exiting until the node has been shutdown
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

