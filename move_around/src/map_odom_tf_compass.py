#!/usr/bin/env python

import sys
import rospy
import tf
from std_msgs.msg import Int8,Float32, Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler



class  MapToOdomCompass():
      
      def __init__(self):
        rospy.init_node('Compass_conversion')
            
        self.robot_name = rospy.get_namespace()
            
        if self.robot_name == "/":
            self.robot_name = ""
            
        self.tf_prefix = rospy.get_param('~tf_prefix',"")
        if self.tf_prefix == "":
		self.tf_prefix = self.tf_prefix
        else:
        	self.tf_prefix = self.tf_prefix +"/"
        self.odom_frame = self.tf_prefix +"odom"
        self.map_frame = "map" 

	self.heading_angle_deg = 0
        
        CompassTopicName = rospy.get_param('~compass_topic_name',"gps_heading")
        CompassTopicName = self.robot_name + CompassTopicName
                
        rospy.Subscriber(CompassTopicName, Float64,  self.CompassCallback)

        CompassCalTopicName = rospy.get_param('~calibrated_compass_topic_name',"calibrated_gps_heading")
        CompassCalTopicName = self.robot_name+CompassCalTopicName
        rospy.Subscriber(CompassCalTopicName,Float64,self.CompassCalCallback)

        self.compass_cal_com = False
        
        
        self.br =tf.TransformBroadcaster()
       
      def CompassCallback(self,msg):
          if self.compass_cal_com == False:
            self.heading_angle_deg = msg.data
      def CompassCalCallback(self,msg):
          if self.compass_cal_com == False:
            self.compass_cal_com = True
            self.heading_angle_deg = msg.data
	
            
if __name__ == '__main__':

      Class_smbg = MapToOdomCompass()
    

   
      rate = rospy.Rate(50)
    
      while  not rospy.is_shutdown():
	Class_smbg.br.sendTransform((0,0,0),quaternion_from_euler(0,0,Class_smbg.heading_angle_deg/57.3),rospy.Time.now(),Class_smbg.map_frame,Class_smbg.odom_frame)
                 
                      
        rate.sleep()
