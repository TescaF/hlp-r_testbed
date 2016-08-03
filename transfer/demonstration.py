#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import sys
import time
import actionlib
import rosbag

from hlpr_kinesthetic_teaching.kinesthetic_interaction import KinestheticInteraction 
from hlpr_kinesthetic_teaching.srv import KinestheticInteract
from hlpr_kinesthetic_teaching.msg import RecordKeyframeDemoAction, RecordKeyframeDemoGoal, RecordKeyframeDemoResult, RecordKeyframeDemoFeedback
from std_msgs.msg import Int32, String

def initGlobals():
  global _initialized

  _initialized = True

class Demonstration(KinestheticInteraction):
  def __init__(self):
    # Initialize the node
    super(Demonstration, self).__init__()
    self.setup()
    rospy.spin()

  def setup(self):
    self.filename = "/home/tesca/data/demos/demo_traj"
    self.finished = False

    #start demo recording
    self.pub = rospy.Publisher('record_demo_frame', Int32, queue_size=10)
    self.client = actionlib.SimpleActionClient('record_keyframe_demo', RecordKeyframeDemoAction)
    self.client.wait_for_server()

    #start kinesthetic mode
    rospy.wait_for_service('kinesthetic_interaction')
    self.teachingMode = rospy.ServiceProxy('kinesthetic_interaction', KinestheticInteract)
    self.teachingMode(True)

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
    goal = RecordKeyframeDemoGoal()
    goal.bag_file_name = self.filename + "_" + str(self.trajNum)
    self.client.send_goal(goal, feedback_cb=self.feedback_call)
    self.pub.publish(0)

  def demonstration_keyframe(self, cmd):
    print "keyframe"

  def demonstration_end(self, cmd):
    #end current trajectory
    print "end"
    self.pub.publish(3)
    self.client.wait_for_result()
    print('[Result] State: %d'%(self.client.get_state()))
    print('[Result] Status: %s'%(self.client.get_goal_status_text()))
    print('[Result] Num Poses Recorded: %f'%(self.client.get_result().num_keyframes))
    self.teachingMode(False)
    self.finished = True

  def demonstration_start_trajectory(self, cmd):
    #end current trajectory
    print "next trajectory"
    print cmd
    self.pub.publish(3)
    self.client.wait_for_result()
    print('[Result] State: %d'%(self.client.get_state()))
    print('[Result] Status: %s'%(self.client.get_goal_status_text()))
    print('[Result] Num Keyframes Recorded: %f'%(self.client.get_result().num_keyframes))

    #start next trajectory
    self.trajNum += 1
    goal = RecordKeyframeDemoGoal()
    goal.bag_file_name = self.filename + "_" + str(self.trajNum)
    self.client.send_goal(goal, feedback_cb=self.feedback_call)
    self.pub.publish(0)

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
    self.teachingMode = rospy.ServiceProxy('kinesthetic_interaction', KinestheticInteract)

  def execute(self, userdata):
    demo = Demonstration()
    while not demo.finished:
      time.sleep(0.1)
    
    userdata.statusOut = 'succeeded'
    return 'succeeded'

if __name__== "__main__":
  demo = Demonstration()
