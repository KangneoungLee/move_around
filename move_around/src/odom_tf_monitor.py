#!/usr/bin/env python

import sys
import rospy
import tf
from std_msgs.msg import Int8,Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler



class  SetMonitorOdom():
      
      def __init__(self):
        rospy.init_node('Odom_tf_monitor')
            
        self.robot_name = rospy.get_namespace()
            
        if self.robot_name == "/":
            self.robot_name = ""
        
	self.tf_prefix = rospy.get_param('~tf_prefix',"")
        if self.tf_prefix == "":
		self.tf_prefix = self.tf_prefix
        else:
        	self.tf_prefix = self.tf_prefix +"/"
        OdomTopicName = rospy.get_param('~odom_topic_name',"odom")
        OdomTopicName = self.robot_name + OdomTopicName
            
        rospy.Subscriber(OdomTopicName, Odometry,  self.OdomMonitorCallback)
        
        
        self.map_frame_name = rospy.get_param('~map_frame_name',"map")
        self.map_frame_name =  self.map_frame_name
        self.base_link_frame_name = rospy.get_param('~base_link_frame_name',"base_link")
        self.base_link_frame_name =  self.tf_prefix + self.base_link_frame_name
        
        self.listener = tf.TransformListener()
           

       
      def OdomMonitorCallback(self,msg):
        
        odom_position_x = msg.pose.pose.position.x
        odom_position_y = msg.pose.pose.position.y
        odom_position_z = msg.pose.pose.position.z
        
        odom_quaternion_x = msg.pose.pose.orientation.x
        odom_quaternion_y = msg.pose.pose.orientation.y
        odom_quaternion_z = msg.pose.pose.orientation.z
        odom_quaternion_w = msg.pose.pose.orientation.w
        
        quaternion_list =[odom_quaternion_x, odom_quaternion_y, odom_quaternion_z, odom_quaternion_w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)  # roll pitch yaw unit : radian
 
        #test_euler_list = 0.519146
        #q_test =quaternion_from_euler(0.0, 0.0, 60/57.3)
        #rospy.loginfo("q_x: %f, q_y: %f, q_z :%f, q_w:%f",q_test[0], q_test[1], q_test[2],q_test[3])
             #The final addition, rospy.spin() simply keeps your node from exiting until the node has been shutdown. 
             #Unlike roscpp, rospy.spin() does not affect the subscriber callback functions, as those have their own threads.


        rospy.loginfo("pos_x odom : %f, pos_y odom: %f, pos_z odom : %f, heading angle odom : %f",odom_position_x,odom_position_y,  odom_position_z,yaw*57.3)
        (trans, rot) = self.listener.lookupTransform(self.map_frame_name,self.base_link_frame_name,rospy.Time(0))
        try:    
          (roll_tf, pitch_tf, yaw_tf) = euler_from_quaternion(rot)
          rospy.loginfo("pos_x tf : %f, pos_y tf: %f, pos_z tf : %f, heading angle tf : %f",trans[0],trans[1],trans[2],yaw_tf*57.3)      
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          print "except"
          #empty


            
if __name__ == '__main__':

      Class_smbg = SetMonitorOdom()
    

   
      rate = rospy.Rate(10)
    
      while  not rospy.is_shutdown():
                 
                      
        rate.sleep()
