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

class Transfer:
  def __init__(self, source, target, mapping):
    self.src = source
    self.tgt = target
    self.mapping = mapping

  def applyMapping(self, targeters, mapping):
    newTargeters = []
    for t in targeters:
      for pair in mapping:
	newT = t.replace(pair[0],pair[1]+"r")
      for pair in mapping:
	newT.replace(pair[1]+"r", pair[1])
      newTargeters.append(newT)
    return newTargeters

  def transfer(self):
    self.tgt.targeters = applyMapping(self.src.targeters, self.mapping)
    self.tgt.dmps = self.src.dmps
    return self.tgt

class TransferState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded'],
                         input_keys=['sourceCaseIn','targetCaseIn','mappingIn'],
                         output_keys=['targetCaseOut'])
    if not _initialized:
      initGlobals()

  def execute(self, userdata):
    tr = Transfer(userdata.sourceCaseIn, userdata.targetCaseIn, userdata.mappingIn)
    newTarget = tr.transfer()
    userdata.targetCaseOut = newTarget
    return 'succeeded'

