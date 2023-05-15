#!/usr/bin/env python 
import rospy 
from geometry_msgs.msg import Twist 
#This class will publish the speed to the /cmd_vel topic to make the robot move for some period of time
# Then stops 
class MoveFClass(): 
    def __init__(self): 
        ###******* INIT PUBLISHERS *******### 
        # create the publisher to cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        my_twist = Twist() # create a twist message, fill in the details
        
        period = 5.0 #How much time the robot will move [seconds]
        rate = rospy.Rate(50) # The rate of the while loop will be 50Hz
        rospy.loginfo("About to be moving forward!")
        while rospy.get_time() == 0:
            print("no simulated time has been received yet")
        start_time = rospy.get_time()  #Get the current time in float seconds
    
        while not rospy.is_shutdown():
            
            if (rospy.get_time() - start_time) <= period: # If we haven't reached the desired "period" of time [s] then move.
                rospy.loginfo("moving forward!")
                # Fill in the message with the required data
                # If we move at 0.2 m/s for 5.0 seconds we will move in the end 1m. 
                my_twist.linear.x = 0.0   # our forward speed in [m/s]. (0.2[m/s]*5[s]) = 1[m]
                my_twist.angular.z = 0.2    # Our angular speed in [rad/s], (In this case the robot does not rotate)                     
            else:
                rospy.loginfo("stop!")
                my_twist.linear.x = 0.0    # our forward speed in [m/s]; 0 => stops the robot.
                my_twist.angular.z = 0.0
            self.cmd_vel_pub.publish(my_twist) # Send the speed to the robot
            # wait enough time to keep the required rate (50Hz)
            rate.sleep()
   
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    # first thing, init a node!
    rospy.init_node('move_forward_some_time') 
    MoveFClass() 
