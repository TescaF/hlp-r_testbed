import roslib
import rospy
import smach
import smach_ros

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import *
from geometry_msgs.msg import *

class NavigationGoal(object):
  
  def __init__(self, withRobot=True):
    #Enable/disable listening and publishing to servers
    self.online = withRobot 

    if(self.online):   
      self.move_base = actionlib.SimpleActionClient("vector_move_base", MoveBaseAction)
      rospy.loginfo("Waiting for move_base action server...")
      self.move_base.wait_for_server()
      
    rospy.loginfo("Connected to move base server")
    rospy.loginfo("Starting navigation")
    rospy.loginfo("The end")
    
  def setGoalPose(self, pose):
    goal = MoveBaseGoal()
    # Use the map frame to define goal poses
    goal.target_pose.header.frame_id = 'map'

    # Set the time stamp to "now"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the goal there
    goal.target_pose.pose.position.x=pose.position.x
    goal.target_pose.pose.position.y=pose.position.y
    goal.target_pose.pose.position.z=0
    
    goal.target_pose.pose.orientation.x=pose.orientation.x
    goal.target_pose.pose.orientation.y=pose.orientation.y
    goal.target_pose.pose.orientation.z=pose.orientation.z
    goal.target_pose.pose.orientation.w=pose.orientation.w
    
    print
    print "Navigation goal pose : "+str(goal.target_pose.pose.position.x)+", "+str(goal.target_pose.pose.position.y)+", "+str(goal.target_pose.pose.position.z)+" Orientation : "+str(goal.target_pose.pose.orientation.x)+", "+str(goal.target_pose.pose.orientation.y)+", "+str(goal.target_pose.pose.orientation.z)+", "+str(goal.target_pose.pose.orientation.w)
    print 
    
    if(self.online):
      # Send the goal pose to the MoveBaseAction server
      self.move_base.send_goal(goal)

      # Allow 1 minute to get there
      finished_within_time = self.move_base.wait_for_result(rospy.Duration(120)) 

      # If we don't get there in time, abort the goal
      if not finished_within_time:
          rospy.loginfo("Timed out achieving goal")
          return False
      else:
          state = self.move_base.get_state()
      if state == GoalStatus.SUCCEEDED:
          rospy.loginfo("Goal succeeded!")
          return True
      else:
          return False
    else:
      return True
