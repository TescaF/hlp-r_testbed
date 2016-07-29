#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import sys
import time

def initGlobals():
  global _initialized

  _initialized = True

class Execution:
  def __init__(self, tgt):
    self.tgt = tgt
    self.action = ActionUtils()
    self.eePose = None
    rospy.Subscriber("eef_publisher", Pose, poseCb)

  def poseCb(self, msg):
    pos = msg.position
    orien = msg.orientation
    self.eePose = [pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w]

  def execute(self):
    self.action.resetPose()

    tol = [0.1,0.1,0.1,100,100,100]
    skill = 0
    for dmp in tgt.dmps:
      init = self.eePose
      goal = tgt.evalTarget(skill)
      goal = goal + [0,0,0]
      traj = self.action.getDMPJointTrajectory(dmp, init, goal, tol) 
      self.action.sendMoveItPlan(traj)
      skill += 1

class ExecutionState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded'],
                         input_keys=['tgtCaseIn'],
                         output_keys=['statusOut'])
    if not _initialized:
      initGlobals()

  def execute(self, userdata):
    exe = Execution(userdata.tgtCaseIn)
    exe.execute();
    userdata.statusOut = 'succeeded'
    return 'succeeded'

