#!/usr/bin/env python

import sys
import rospy
import tf
from std_msgs.msg import Int8,Float32, Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler



class  CompassProc():
      
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
        self.baselink_frame = self.tf_prefix + "/" +"base_link" 
        CompassProcResetName = rospy.get_param('~compass_proc_reset_topic_name',"compass_proc_reset")
        CompassProcResetName = self.robot_name + CompassProcResetName
        rospy.Subscriber(CompassProcResetName, Int8,  self.reset_callback)
        
        CompassTopicName = rospy.get_param('~compass_topic_name',"global_position/compass_hdg")
        CompassTopicName = self.robot_name + CompassTopicName
        
        self.heading_angle_deg = 0
        self.heading_angle_deg_temp = 0
        self.angstab_revflag = False
        self.angrev_time = rospy.Time.now()
        self.heading_angle_rad = 0
        
        rospy.Subscriber(CompassTopicName, Float64,  self.CompassCallback)
        
        
        VirImuTopicName = rospy.get_param('~virtual_imu_topic_name',"virtual_imu")
        VirImuTopicName = self.robot_name + VirImuTopicName
        
        self.virimu_pub = rospy.Publisher(VirImuTopicName,Imu, queue_size=1)

        ComHeadCalTopicName = rospy.get_param('~calibrated_compass_topic_name',"gps_heading_calibrated")
        ComHeadCalTopicName = self.robot_name + ComHeadCalTopicName
        
        self.comheadcal_pub = rospy.Publisher(ComHeadCalTopicName,Float64, queue_size=1)
         
         
        self.calibration_on_flag = rospy.get_param('~use_compass_calibration',False)
         
        if  self.calibration_on_flag == False:
           self.calibration_compflag = True
           self.compass_offset =  rospy.get_param('~compass_manual_offset',0)
        else:
           self.calibration_compflag = False
           self.compass_offset = 0
           
        self.compass_std_align =  rospy.get_param('~initial_orientation_for_compass_cal',0)
        
        self.stability_check_flag = rospy.get_param('~use_compass_stability_check',True)
        
        if self.stability_check_flag == False:
           self.compass_stable_flag = True
        else :
           self.compass_stable_flag = False
        
        self.stability_check_bound = rospy.get_param('~compass_stability_check_bound',2)
        self.stability_check_duration = rospy.get_param('~compass_stability_check_duration',10)
           

       
      def CompassCallback(self,msg):
        
        self.heading_angle_deg = msg.data
        heading_angle_deg_print = self.heading_angle_deg 
        
        if self.stability_check_flag == True and self.compass_stable_flag == False:
            self.compass_stab_check()
        
        if self.calibration_on_flag == True and self.compass_stable_flag == True and  self.calibration_compflag == False:
            
            
            self.compass_cal()
            rospy.loginfo("compass calibration sucess, before heading : %f, after heading : %f",heading_angle_deg_print,self.heading_angle_deg + self.compass_offset)
        
        self.heading_angle_rad = (self.heading_angle_deg + self.compass_offset)/57.3
        heading_angle_quat=quaternion_from_euler(0.0, 0.0, self.heading_angle_rad)
        
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = self.odom_frame
        imu_msg.orientation.x = heading_angle_quat[0]
        imu_msg.orientation.y = heading_angle_quat[1]
        imu_msg.orientation.z = heading_angle_quat[2]
        imu_msg.orientation.w = heading_angle_quat[3]
        
        if self.calibration_compflag and self.compass_stable_flag == True:
        
            self.virimu_pub.publish(imu_msg)
	    self.comheadcal_pub.publish(self.heading_angle_deg + self.compass_offset)
            rospy.loginfo("compass calibration sucess, before heading : %f, after heading : %f",heading_angle_deg_print,self.heading_angle_deg + self.compass_offset)
            
            
            
            
      def compass_cal(self):
      
        self.compass_offset = -self.heading_angle_deg  + self.compass_std_align
        self.calibration_compflag = True
      
      def compass_stab_check(self):
      
        if  self.angstab_revflag == False :
            self.heading_angle_deg_temp = self.heading_angle_deg
            self.angrev_time = rospy.Time.now()
            self.angstab_revflag = True
            
        heading_angle_error = 180
        
        if  (rospy.Time.now() - self.angrev_time) > rospy.Duration(self.stability_check_duration) and self.angstab_revflag == True:
            heading_angle_error   = self.heading_angle_deg_temp - self.heading_angle_deg
            self.angstab_revflag = False
        
        if heading_angle_error < self.stability_check_bound and heading_angle_error > -self.stability_check_bound:
            rospy.loginfo("compass stability check sucess, time duration : %f, compass error bound : %f",self.stability_check_duration,heading_angle_error)
            self.compass_stable_flag = True
        elif (rospy.Time.now() - self.angrev_time) > rospy.Duration(self.stability_check_duration):
            rospy.loginfo("compass stability check fail, time duration : %f, compass error bound : %f",self.stability_check_duration,heading_angle_error)
            self.compass_stable_flag = False
      
      def reset_callback(self, msg):
        
        rospy.loginfo("reset flag received")
        self.angstab_revflag = False
        self.angrev_time = rospy.Time.now()
        if self.stability_check_flag == False:
           self.compass_stable_flag = True
        else :
           self.compass_stable_flag = False
           
        if  self.calibration_on_flag == False:
           self.calibration_compflag = True
           self.compass_offset =  rospy.get_param('~compass_manual_offset',0)
        else:
           self.calibration_compflag = False
           self.compass_offset = 0
        

            
if __name__ == '__main__':

      Class_smbg = CompassProc()
    

   
      rate = rospy.Rate(10)
    
      while  not rospy.is_shutdown():
                 
                      
        rate.sleep()
