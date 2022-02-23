#!/usr/bin/env python

import sys
import rospy
import actionlib
from visitgpsclient.msg import VisitPositionsRobotGPSAction,  VisitPositionsRobotGPSGoal, VisitPositionsRobotGPSResult
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32



class  SetVisitGpsGoal():
      
      def __init__(self):
            rospy.init_node('set_visit_gps_goal')
            self.visit_gps = actionlib.SimpleActionClient('groundVisitPositionsRobotGPS', VisitPositionsRobotGPSAction)
            self.visit_gps.wait_for_server()
            
            
if __name__ == '__main__':

    Class_smbg = SetVisitGpsGoal()
    
    theta = 0

    i = 0
   
    while  not rospy.is_shutdown():
          
            if  not sys.argv[1] == None and not sys.argv[2] == None and not i == 1 :
                 
                 gps_goal_temp = NavSatFix()
                 heading_input_deg_temp = Float32()
                 times_to_stay_at_positions_temp = Float32()
                 
                 gps_goal_temp.latitude = float(sys.argv[1])
                 gps_goal_temp.longitude = float(sys.argv[2])
                 
                 heading_input_deg_temp.data = 2.0
                 times_to_stay_at_positions_temp.data = 5.1
                 
                 #quaternion = Quaternion(*quaternion_from_euler(0, 0, ((theta * math.pi) / 180)))
               
                 goal = VisitPositionsRobotGPSGoal()
                 
                 goal.positions = gps_goal_temp,
                 #goal.heading_input_deg = heading_input_deg_temp,
                 #goal.times_to_stay_at_positions = times_to_stay_at_positions_temp,

                 
                 Class_smbg.visit_gps.send_goal(goal)
                 Class_smbg.visit_gps.wait_for_result()   
                 i = i +1
             
            rospy.spin()
