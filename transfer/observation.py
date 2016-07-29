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

class Observation:
  def __init__(self, directory):
    self.directory = directory
    self.features = None
    self.labels = None
    self.knowledgeLabels = None
    self.properties = None
    self.affordances = None

    rospy.Subscriber("beliefs/labels", LabeledFeatureArray, labelCb)
    rospy.Subscriber("beliefs/knowledge", ObjectKnowledge, knowCb)

  def labelCb(self, msg):
    self.features = msg.features
    self.labels = msg.labels

  def knowCb(self, msg):
    self.knowledgeLabels = msg.labels
    self.properties = msg.properties
    self.affordances = msg.affordances

  def indexOf(self, name, arr):
    i = 0
    for a in arr:
      if name == arr:
	return i
      i += 1
    return -1

  def getNewCaseName(self):
    i = 0
    while True:
      dirName = self.directory + "c" + str(i)
      if os.path.isdir(dirName)
	return dirName
      i += 1

  def writeCase(self):
    name = self.getNewCaseName()
    objs = self.getObjects()
    c = Case(self.directory, name, None, None, objs)
    return c

  def getObjects(self):
    while self.features is None or self.labels is None or self.properties is None or self.affordances is None:
	time.wait(1)
    objects = []
    i = 0
    for feats in self.features:
	label = self.labels[i]
	loc = [feats.points_centroid.x, feats.points_centroid.y, feats.points_centroid.z]
	dim = [feats.bb_dims.x, feats.bb_dims.y, feats.bb_dims.z]
	hue = feats.hue
        idx = self.indexOf(label, self.knowledgeLabels)
	aff = self.affordances[idx]
	prop = self.properties[idx]
	o = TableObject(label, loc, dim, hue, aff, prop)
	objects.append(o)
	i += 1
    return objects

class ObservationState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded'],
                         input_keys=['directoryIn'],
                         output_keys=['targetCaseOut'])
    if not _initialized:
      initGlobals()

  def execute(self, userdata):
    obs = Observation(userdata.directoryIn)
    userdata.targetCaseOut = obs.writeCase()
    return 'succeeded'

