#!/usr/bin/env python

import sys
import rospy
import actionlib
from visitgpsclient.msg import VisitPositionsRobotGPSAction,  VisitPositionsRobotGPSGoal, VisitPositionsRobotGPSResult
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8,Float32
from geometry_msgs.msg import Twist



class  SetVisitGpsGoal():
      
      def __init__(self):
            rospy.init_node('set_visit_gps_goal')

            self.robot_name = rospy.get_namespace()
            print "debug"
            if (self.robot_name == "/"):
                self.robot_name = ""
            
            self.using_waypoint = rospy.get_param('~using_wapoint_for_gps_input',False)
            self.linear_base_speed = rospy.get_param('~linear_base_speed',"0.5")
            self.angular_base_speed = rospy.get_param('~angular_base_speed',"2.0")

            CmdVelTopicName = self.robot_name + rospy.get_param('~cmd_vel_topic_name',"cmd_vel")
            UnitySimple_CmdTopicName = self.robot_name + rospy.get_param('~unity_simple_int8_topic_name',"Command_unity_sender")
            UnityGps_CmdTopicName = self.robot_name + rospy.get_param('~unity_gps_cmd_topic_name',"gps_unity_sender")
            SpeedMul_TopicName =  self.robot_name + rospy.get_param('~speed_multiplier_topic_name',"Speed_unity_sender")
            
            Waypoint_comp_TopicName = self.robot_name + rospy.get_param('~waypoint_complete_topic_name',"Waypoint_comp_flag")
            
            self.waypoint_comp_pub = rospy.Publisher(Waypoint_comp_TopicName,Int8, queue_size=10)
            
            
            self.cmd_pub = rospy.Publisher(CmdVelTopicName,Twist, queue_size=10)
            
            self.visit_gps = actionlib.SimpleActionClient('groundVisitPositionsRobotGPS', VisitPositionsRobotGPSAction)
            #self.visit_gps.wait_for_server()
            
            self.unitycmd_rev = 0
            self.unitycmd_lat = 0
            self.unitycmd_long = 0

            self.linear_speed_multiplier = 1
            self.angular_speed_multiplier = 1
            
            rospy.Subscriber(UnitySimple_CmdTopicName, Int8,  self.UnitySimple_CmdTopicCallback, queue_size = 1)
           
            if self.using_waypoint == False :
                rospy.Subscriber(UnityGps_CmdTopicName, NavSatFix,  self.UnityGps_CmdTopicCallback)
            else :
                rospy.Subscriber(UnityGps_CmdTopicName, NavSatFix,  self.UnityGps_CmdTopicCallback_list)
             
            rospy.Subscriber(SpeedMul_TopicName, Float32,  self.SpeedMul_TopicCallback)
            
            self.latitude_list =[]
            self.longitude_list =[]
            
            

           

       
      def UnitySimple_CmdTopicCallback(self,msg):
         
         #self.UnitySimple_CmdValue = Int8()
         self.UnitySimple_CmdValue = msg.data;
            
         self.cmd_vel = Twist()
         rospy.loginfo("Unity simple command input is %d",self.UnitySimple_CmdValue)
            
         if(self.UnitySimple_CmdValue == 1):
             rospy.loginfo("Unity simple command is 1 and forward motion")
             self.cmd_vel.linear.x = self.linear_base_speed*self.linear_speed_multiplier
             self.cmd_pub.publish(self.cmd_vel)
            
         elif(self.UnitySimple_CmdValue == 2):
             rospy.loginfo("Unity simple command is 2 and backward motion")
             self.cmd_vel.linear.x = -self.linear_base_speed*self.linear_speed_multiplier
             self.cmd_pub.publish(self.cmd_vel)

         elif(self.UnitySimple_CmdValue == 3):
             rospy.loginfo("Unity simple command is 3 and CCW rot motion")
             self.cmd_vel.angular.z = self.angular_base_speed*self.angular_speed_multiplier
             self.cmd_pub.publish(self.cmd_vel)

         elif(self.UnitySimple_CmdValue == 4):
             rospy.loginfo("Unity simple command is 4 and CW rot motion")
             self.cmd_vel.angular.z = -self.angular_base_speed*self.angular_speed_multiplier
             self.cmd_pub.publish(self.cmd_vel)

         elif(self.UnitySimple_CmdValue == 5):
             rospy.loginfo("Unity simple command is 5 and stop motion")
             self.cmd_vel.linear.x = 0
             self.cmd_vel.angular.z = 0
             self.cmd_pub.publish(self.cmd_vel)
             
         
             
             #The final addition, rospy.spin() simply keeps your node from exiting until the node has been shutdown. 
             #Unlike roscpp, rospy.spin() does not affect the subscriber callback functions, as those have their own threads.
      
      def UnityGps_CmdTopicCallback_list(self,msg):
         
         #rospy.loginfo("Unity gps target is received")
         
         if self.unitycmd_rev == 0 :
            self.unitycmd_lat = msg.latitude
            self.unitycmd_long = msg.longitude
         
            if self.unitycmd_lat == -999 and self.unitycmd_long ==-999 :
                if len(self.latitude_list) > 0 :
                    self.unitycmd_rev = 1
            else :
                if  not self.latitude_list and not  self.longitude_list : 
                    self.latitude_list.append(self.unitycmd_lat)
                    self.longitude_list.append(self.unitycmd_long)
                    rospy.loginfo("Unity gps target is intially appended")
                else :
                    if self.unitycmd_lat != self.latitude_list[-1] or self.unitycmd_long != self.longitude_list[-1] : 
                        self.latitude_list.append(self.unitycmd_lat)
                        self.longitude_list.append(self.unitycmd_long)
                        rospy.loginfo("Unity gps target is appended")

         #now = rospy.get_rostime()
         #rospy.loginfo("lat : %f, long : %f, receive flag : %d",self.unitycmd_lat,self.unitycmd_long,self.unitycmd_rev)
         #rospy.loginfo("Time : %i  %i",now.secs,now.nsecs)
         
      def UnityGps_CmdTopicCallback(self,msg):
         
         rospy.loginfo("Unity gps target is received")
         self.unitycmd_lat = msg.latitude
         self.unitycmd_long = msg.longitude
         
         self.unitycmd_rev = 1

         rospy.loginfo("lat : %f, long : %f, receive flag : %d",self.unitycmd_lat,self.unitycmd_long,self.unitycmd_rev)

      def SpeedMul_TopicCallback(self,msg):
         
         if(msg.data>0) :
            self.linear_speed_multiplier = msg.data
         elif(msg.data<0) :
            self.angular_speed_multiplier = -msg.data

         rospy.loginfo("linear speed multiplier = %f, angular speed multiplier = %f",self.linear_speed_multiplier,self.angular_speed_multiplier)


            
if __name__ == '__main__':

    Class_smbg = SetVisitGpsGoal()
    
    theta = 0

    i = 0
   
    rate = rospy.Rate(10)
    
    while  not rospy.is_shutdown():
                 

         heading_input_deg_temp = Float32()
         times_to_stay_at_positions_temp = Float32()
               
         heading_input_deg_temp.data = 2.0
         times_to_stay_at_positions_temp.data = 5.1
                 
         #quaternion = Quaternion(*quaternion_from_euler(0, 0, ((theta * math.pi) / 180)))
               

         #goal.heading_input_deg = heading_input_deg_temp,
         #goal.times_to_stay_at_positions = times_to_stay_at_positions_temp,
         if Class_smbg.using_waypoint == False :
            if(Class_smbg.unitycmd_rev == 1):
         
                gps_goal_temp = NavSatFix()

                 
                gps_goal_temp.latitude = Class_smbg.unitycmd_lat
                gps_goal_temp.longitude = Class_smbg.unitycmd_long
             
                goal = VisitPositionsRobotGPSGoal()
                 
                goal.positions = gps_goal_temp,
             
             
                Class_smbg.visit_gps.wait_for_server()
                Class_smbg.visit_gps.send_goal(goal)
             
                rospy.loginfo("Gps target is sent")             
    
                Class_smbg.visit_gps.wait_for_result()
                Int8_temp = Int8()
                Int8_temp.data = 1
                Class_smbg.waypoint_comp_pub.publish(Int8_temp)
                rospy.loginfo("Action result is received")   
         
                Class_smbg.unitycmd_rev = 0
         
         else :
            if(Class_smbg.unitycmd_rev == 1):
               
                for index in range(0,len(Class_smbg.latitude_list)) :
                    gps_goal_temp = NavSatFix()
                    
                    gps_goal_temp.latitude = Class_smbg.latitude_list[index]
                    gps_goal_temp.longitude = Class_smbg.longitude_list[index]
                    
                    #rospy.loginfo("Gps target is sent latitude : %f  longitude : %f",gps_goal_temp.latitude,gps_goal_temp.longitude)
                     
                    goal = VisitPositionsRobotGPSGoal() 
                    goal.positions = gps_goal_temp,
                    Class_smbg.visit_gps.wait_for_server()
                    Class_smbg.visit_gps.send_goal(goal)
                    rospy.loginfo("Gps target is sent %d/%d (current / total)",index+1,len(Class_smbg.latitude_list))
                    
                    Class_smbg.visit_gps.wait_for_result()
                    Int8_temp = Int8()
                    Int8_temp.data = 1
                    Class_smbg.waypoint_comp_pub.publish(Int8_temp)
                    rospy.loginfo("Action result is received %d/%d (current / total)",index+1,len(Class_smbg.latitude_list))   
                    
                Class_smbg.unitycmd_rev = 0
                Class_smbg.latitude_list = []
                Class_smbg.longitude_list = []
                      
         rate.sleep()
