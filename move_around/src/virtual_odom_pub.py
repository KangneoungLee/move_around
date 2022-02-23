#!/usr/bin/env python

import sys
import rospy
import tf
from std_msgs.msg import Int8,Float32, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler



class  VirtualOdomPub():
      
      def __init__(self):
        rospy.init_node('virtual_odom_pub')
            
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

        self.base_link_frame_name = rospy.get_param('~base_link_frame_name',"base_link")
        self.base_link_frame_name =  self.tf_prefix + self.base_link_frame_name


        CompassCalTopicName = rospy.get_param('~calibrated_compass_topic_name',"calibrated_gps_heading")
        CompassCalTopicName = self.robot_name+CompassCalTopicName
        rospy.Subscriber(CompassCalTopicName,Float64,self.CompassCalCallback)

        self.compass_cal_com = False
        
        
        self.listener = tf.TransformListener()

        VirOdomTopicName = rospy.get_param('~virtual_odom_topic_name',"odom_virtual")
        VirOdomTopicName = self.robot_name+VirOdomTopicName

	self.virodom_pub = rospy.Publisher(VirOdomTopicName,Odometry, queue_size=1)


	OdomTopicName = rospy.get_param('~odom_topic_name',"odom")
        OdomTopicName = self.robot_name + OdomTopicName
            
        rospy.Subscriber(OdomTopicName, Odometry,  self.OdomCallback)

      def CompassCalCallback(self,msg):
          if self.compass_cal_com == False:
            self.compass_cal_com = True

      def OdomCallback(self,msg):
	self.odom_vel_x = msg.twist.twist.linear.x
        self.odom_vel_ang_z = msg.twist.twist.angular.z
 

      def TFlisten_and_odompub(self):
	if self.compass_cal_com == True:
	    
		(trans, rot) = self.listener.lookupTransform(self.map_frame,self.base_link_frame_name,rospy.Time(0))
		
	
		Odom_msg = Odometry()

        	Odom_msg.header.stamp = rospy.Time.now()
        	Odom_msg.header.frame_id = self.odom_frame
        	Odom_msg.child_frame_id = self.base_link_frame_name


        	Odom_msg.pose.pose.position.x = trans[0]
        	Odom_msg.pose.pose.position.y = trans[1]
        	Odom_msg.pose.pose.position.z = 0.0
        	Odom_msg.pose.pose.orientation = Quaternion(*rot)
		Odom_msg.twist.twist.linear.x = self.odom_vel_x
		Odom_msg.twist.twist.angular.z = self.odom_vel_ang_z 


		(roll, pitch, yaw) = euler_from_quaternion(rot)

		rospy.loginfo("yaw angle the robot from map frame : %f",yaw*57.3)

		self.virodom_pub.publish(Odom_msg)

	
            
if __name__ == '__main__':

      Class_smbg = VirtualOdomPub()
    

   
      rate = rospy.Rate(10)
    
      while  not rospy.is_shutdown():
	Class_smbg.TFlisten_and_odompub()
                 
                      
        rate.sleep()
