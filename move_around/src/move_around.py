#!/usr/bin/env python

import sys
import rospy
import actionlib
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction,  MoveBaseGoal, MoveBaseResult
#from visitgpsclient.msg import VisitPositionsRobotGPSAction,  VisitPositionsRobotGPSGoal, VisitPositionsRobotGPSResult
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8,Float32
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from rover_motor_driver.srv import *


multiple_goal_position =[[-2.5/3,-3/3],[0/3,0/3],[-2/3,2.5/3],[-5/3,5/3],[-7.5/3,2/3],[0/3,0/3],[-7.5/3,-2.5/3],[-5/3,-5/3]]

class  SetMoveAroundGoal():
      
   def __init__(self):
      rospy.init_node('set_move_around_goal')

      self.tf_prefix = rospy.get_param('~tf_prefix',"")

      if(self.tf_prefix != "") :
         self.tf_prefix = self.tf_prefix +"/"
      
      self.map_frame = rospy.get_param('~map_frame',"map")
      self.robot_frame = self.tf_prefix+rospy.get_param('~robot_base_frame',"base_link")
      self.action_result_wait_duration = rospy.get_param('~wait_duration',20)
      #self.using_waypoint = rospy.get_param('~using_wapoint_for_gps_input',False)

      self.cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=1)

      rospy.Subscriber('move_around/start', Int8,  self.StartTopicCallback, queue_size = 1)
      rospy.Subscriber('move_around/pause', Int8,  self.PauseTopicCallback, queue_size = 1)
      rospy.Subscriber('move_around/reset', Int8,  self.ResetTopicCallback, queue_size = 1)
      
      self.move_around_srv = rospy.ServiceProxy('wheel_odompose_srv',WheelOdomSet)
            
      self.move_base_action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            
      self.goal_index = 0
      self.pause_loop = True

      self.rate = rospy.Rate(10)

   def StartTopicCallback(self,msg):
      pose_set = PoseWithCovarianceStamped()
      pose_set.pose.pose.position.x = 0
      pose_set.pose.pose.position.y = 0
      pose_set.pose.pose.position.z = 0
      pose_set.pose.pose.orientation.x = 0
      pose_set.pose.pose.orientation.y = 0
      pose_set.pose.pose.orientation.z = 0
      pose_set.pose.pose.orientation.w = 1
       
      rospy.loginfo("*********wait for the wheel odometry service ******")
      rospy.wait_for_service('wheel_odompose_srv')
      rospy.loginfo("*********the wheel odometry service connected******")

      resp = self.move_around_srv(pose_set)

      if resp.result == True:
         rospy.loginfo("*********Success wheel odometry set******")
      else:
         rospy.logwarn("*********Fail wheel odometry set******")

      self.pause_loop = False
      rospy.loginfo("*********start the move around ******")

   def ResetTopicCallback(self,msg):

      cancel_msg = GoalID()
      self.cancel_pub.publish(cancel_msg)
      rospy.loginfo("*********Reset the move around ******")
      self.goal_index = 0

      if self.pause_loop == True :
         rospy.loginfo("*********You shoud restart the move around by publishing the 'move_around/start' topic with any value ******")

   def PauseTopicCallback(self,msg):

      self.pause_loop = True
      cancel_msg = GoalID()
      self.cancel_pub.publish(cancel_msg)
      rospy.loginfo("*********Pause the move around ***** to start move around, publish the 'move_around/start' topic with any value ******")
      #if msg.data >= 1 :
      #   self.pause_loop = True
      #   cancel_msg = GoalID()
      #   self.cancel_pub.publish(cancel_msg)
      #   rospy.loginfo("*********Pause the move around ******")
      #else : 
      #   self.pause_loop = False
      #   rospy.loginfo("*********Restart the move around ******")


   def run(self):

      goal = MoveBaseGoal()

      while not rospy.is_shutdown() :
 
         if self.pause_loop == True :
            continue 

         rospy.loginfo("*********start to detect move base action server******")
         self.move_base_action_client.wait_for_server()
         rospy.loginfo("*********move base action server detected******")
         Server_active_time = rospy.Time.now()

         goal.target_pose.header.frame_id = self.map_frame
         goal.target_pose.header.stamp = rospy.Time.now()

         goal.target_pose.pose.position.x = multiple_goal_position[self.goal_index][0]
         goal.target_pose.pose.position.y = multiple_goal_position[self.goal_index][1]
         
         goal_heading_angle = 0 #unit L rad

         q = quaternion_from_euler(0, 0, goal_heading_angle)
         goal.target_pose.pose.orientation.x = q[0]
         goal.target_pose.pose.orientation.y = q[1]
         goal.target_pose.pose.orientation.z = q[2]
         goal.target_pose.pose.orientation.w = q[3]

         #goal.target_pose.pose.orientation.x = 0
         #goal.target_pose.pose.orientation.y = 0
         #goal.target_pose.pose.orientation.z = 0
         #goal.target_pose.pose.orientation.w = 1

         self.move_base_action_client.send_goal(goal)
         rospy.loginfo("*********send move base goal ******")
         rospy.loginfo("*********index of goal : %d ____goal position x : %f ____  goal position y : %f ******",self.goal_index,goal.target_pose.pose.position.x,goal.target_pose.pose.position.y)

         wait_result = self.move_base_action_client.wait_for_result(rospy.Duration(self.action_result_wait_duration))

         rospy.loginfo("*********  wait result : %d ******",wait_result)

         if (wait_result == False) :
            #send a cancel goal command and move to next goal 
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)

            self.goal_index = self.goal_index + 1

            rospy.loginfo("*********cancel the goal because of the timeout, go to the next goal ******")

         else :  #wait_result == True
            #only move to the next goal
            if(wait_result>0) :
               rospy.loginfo("*********fail because of the move base error, go to the next goal ******")  
            else :
               rospy.loginfo("*********success, go to the next goal ******") #move_base/result msg.state.status = 3

            self.goal_index = self.goal_index + 1

            

         
         if self.goal_index == len(multiple_goal_position) :
            rospy.loginfo("*********send all goals, go to the initial goal ******")
            self.goal_index = 0  


         while self.pause_loop == True :
            dummy = 1
            ### empty loop ###

            
         self.rate.sleep()
           
            
if __name__ == '__main__':

    Class_smbg = SetMoveAroundGoal()
    Class_smbg.run()
       
 
                      

