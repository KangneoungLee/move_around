#!/usr/bin/env python

import sys
import rospy
import actionlib
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction,  MoveBaseGoal, MoveBaseResult
#from visitgpsclient.msg import VisitPositionsRobotGPSAction,  VisitPositionsRobotGPSGoal, VisitPositionsRobotGPSResult
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8,Float32
from geometry_msgs.msg import Twist


multiple_goal_position =[[5,4],[10,10],[6,15],[0,20],[-5,14],[-10,-10],[-5,-5],[0,0]]

class  SetMoveAroundGoal():
      
   def __init__(self):
      rospy.init_node('set_move_around_goal')

      self.tf_prefix = rospy.get_param('~tf_prefix',"")
      self.robot_frame = self.tf_prefix+rospy.get_param('~robot_base_frame',"base_link")
      self.action_result_wait_duration = rospy.get_param('~wait_duration',20)
      #self.using_waypoint = rospy.get_param('~using_wapoint_for_gps_input',False)

      self.cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=1)

      rospy.Subscriber("move_around/pause, Int8,  self.PauseTopicCallback, queue_size = 1)
      rospy.Subscriber("move_around/reset, Int8,  self.ResetTopicCallback, queue_size = 1)
            
      self.move_base_action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            
      self.goal_index = 0
      self.pause_loop = False

      self.rate = rospy.Rate(10)

   def ResetTopicCallback(self,msg):
      if msg.data >= 1 :
         cancel_msg = GoalID()
         self.cancel_pub.publish(cancel_msg)
         rospy.loginfo("*********Reset the move around ******")
         self.goal_index = 0

         if self.pause_loop == True :
            rospy.loginfo("*********You shoud restart the move around by publishing the 'move_around/pause' topic with 0 value ******")

   def PauseTopicCallback(self,msg):
      if msg.data >= 1 :
         self.pause_loop = True
         cancel_msg = GoalID()
         self.cancel_pub.publish(cancel_msg)
         rospy.loginfo("*********Pause the move around ******")
      else : 
         self.pause_loop = False
         rospy.loginfo("*********Restart the move around ******")


   def run(self):

      goal = MoveBaseGoal()

      while not rospy.is_shutdown() : 

         self.move_base_action_client.wait_for_server()
         rospy.loginfo("*********detect move base action server******")
         Server_active_time = rospy.Time.now()

         goal.target_pose.header.frame_id = self.robot_frame
         goal.target_pose.header.stamp = rospy.Time.now()

         goal.target_pose.pose.position.x = multiple_goal_position[i][0]
         goal.target_pose.pose.position.y = multiple_goal_position[i][1]

         self.move_base_action_client.send_goal(goal)
         rospy.loginfo("*********send move base goal ******")
         rospy.loginfo("*********index of goal : %d ____goal position x : %f ____  goal position y : %f ******",self.goal_index,goal.target_pose.pose.position.x,goal.target_pose.pose.position.y)

         wait_result = self.move_base_action_client.wait_for_result(rospy.Duration(self.action_result_wait_duration))

         if (wait_result == False) :
            #send a cancel goal command and move to next goal 
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)

            self.goal_index = self.goal_index + 1

            rospy.loginfo("*********cancel the goal because of the timeout, go to the next goal ******")

         else :  #wait_result == True
            #only move to the next goal

            self.goal_index = self.goal_index + 1

            rospy.loginfo("*********success, go to the next goal ******")

         
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
       
 
                      

