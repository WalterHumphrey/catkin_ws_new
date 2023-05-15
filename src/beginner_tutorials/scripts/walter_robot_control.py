#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

class RobotControl():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        # Topico donde publicaremos la velocidad con mensajes tipo twist
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # Topico que escucharemos para recibir la orden
        rospy.Subscriber("order", String, self.order_cb)
        # Velocidad a la que publicamos el mensaje
        r = rospy.Rate(10) #10Hz
        # Acceder y modificar la velocidad
        self.robot_vel = Twist()

        while not rospy.is_shutdown():
            self.pub.publish(self.robot_vel)
            r.sleep()

    # Funcion que manda la velocidad dependiendo de la orden recibida
    def order_cb(self, order):
        self.robot_vel.linear.x = 0.0
        self.robot_vel.angular.z = 0.0
        self.order = order.data

        if (self.order == 'forward'):
            self.robot_vel.linear.x = 0.1
            print("test")
        elif (self.order == 'back'):
            self.robot_vel.linear.x = -0.1
        elif (self.order == 'left'):
            self.robot_vel.angular.z = 0.3
        elif (self.order == 'right'):
            self.robot_vel.angular.z = -0.3
        elif (self.order == 'stop'):
            self.robot_vel.linear.x = 0.0
            self.robot_vel.angular.z = 0.0
        elif (self.order == 'tri'):
            for i in range(3):
                self.robot_vel.linear.x = 0.1
                self.robot_vel.angular.z = 0.0
                rospy.sleep(3)
                self.robot_vel.linear.x = 0.0
                self.robot_vel.angular.z = 0.6
                rospy.sleep(3)
            self.robot_vel.linear.x = 0.0
            self.robot_vel.angular.z = 0.0
        return self.robot_vel

 # Funcion de limpieza, para todo movimiento
    def cleanup(self):
        self.robot_vel.linear.x = 0.0
        self.robot_vel.angular.z = 0.0
        self.pub.publish(self.robot_vel)
        return
############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
 rospy.init_node('cmd_vel_control', anonymous=False)
 RobotControl()