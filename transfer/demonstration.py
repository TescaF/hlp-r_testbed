#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import sys
import time
import actionlib
import rosbag

from hlpr_kinesthetic_teaching.kinesthetic_interaction import *
from hlpr_kinesthetic_teaching.msg import RecordKeyframeDemoAction, RecordKeyframeDemoGoal, RecordKeyframeDemoResult, RecordKeyframeDemoFeedback
from std_msgs.msg import Int32, String

def initGlobals():
  global _initialized

  _initialized = True

class Demonstration(KinestheticInteraction):
  def __init__(self):
    # Initialize the node
    super(BasicKinestheticInteraction, self).__init__()
    self.filename = "demo_data"
    self.pub = rospy.Publisher('record_demo_frame', Int32, queue_size=10)
    client = actionlib.SimpleActionClient('record_keyframe_demo', RecordKeyframeDemoAction)
    client.wait_for_server()
    self.finished = false

  def feedback_call(feedback):
    print('[Feedback] num keyframes: %f'%feedback.num_keyframes)

  def apply_hand_action(self, cmd, hand):
    print "hand"

  def apply_arm_action(self, cmd, arm):
    print "arm"

  def demonstration_start(self, cmd):
    #start next trajectory
    print "start"
    self.trajNum = 0
    goal.bag_file_name = filename + "_" + str(self.trajNum)
    client.send_goal(goal, feedback_cb=feedback_call)
    pub.publish(0)

  def demonstration_keyframe(self, cmd):
    print "keyframe"

  def demonstration_end(self, cmd):
    #end current trajectory
    print "end"
    pub.publish(3)
    client.wait_for_result()
    print('[Result] State: %d'%(client.get_state()))
    print('[Result] Status: %s'%(client.get_goal_status_text()))
    print('[Result] Num Keyframes Recorded: %f'%(client.get_result().num_keyframes))
    self.finished = true

  def demonstration_start_trajectory(self, cmd):
    #end current trajectory
    print cmd
    pub.publish(3)
    client.wait_for_result()
    print('[Result] State: %d'%(client.get_state()))
    print('[Result] Status: %s'%(client.get_goal_status_text()))
    print('[Result] Num Keyframes Recorded: %f'%(client.get_result().num_keyframes))

    #start next trajectory
    self.trajNum += 1
    goal.bag_file_name = filename + "_" + str(self.trajNum)
    client.send_goal(goal, feedback_cb=feedback_call)
    pub.publish(0)

  def demonstration_end_trajectory(self, cmd):
    print cmd


class DemonstratonState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded'],
                         input_keys=['goalIn'],
                         output_keys=['statusOut'])
    if not _initialized:
      initGlobals()

  def execute(self, userdata):
    demo = Demonstration()
    
    userdata.statusOut = 'succeeded'
    return 'succeeded'

if __name__== "__main__":
  demo = Demonstration()
