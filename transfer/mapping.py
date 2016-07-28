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

class Retrieval:
  def __init__(self):

class RetrievalState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded'],
                         input_keys=['goalIn'],
                         output_keys=['statusOut'])
    if not _initialized:
      initGlobals()

  def execute(self, userdata):
    userdata.statusOut = 'succeeded'
    return 'succeeded'

