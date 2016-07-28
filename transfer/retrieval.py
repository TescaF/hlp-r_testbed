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
    self.directory = "~/data/case_memory/"

  def loadCase(self):
    name = "c1"
    loc = self.directory + name

    dmps = loadDMPs(loc)
    targeters = loadTargeters(loc)
    objects = loadObjects(loc)
    case = Case(loc, name, dmps, targeters, objects)
    return case

  def loadDMPs(self, location):
    with open(location) as f:
	lines = f.readlines()
    dmps = []
    for i in range(0, len(lines), 5):
	dmp = DMPData()
	dmp.k_gain = float(lines[i])
	dmp.d_gain = float(lines[i+1])
	dmp.weights = map(float,lines[i+2].split(",")
	dmp.f_domain = map(float,lines[i+3].split(",")
	dmp.f_targets = map(float,lines[i+4].split(",")
	dmps.append(dmp)
    return dmps

  def loadTargeters(self, location):
    with open(location) as f:
        lines = f.readlines()
    return lines

  def loadObjects(self, location):
    with open(location) as f:
        lines = f.readlines()
    objects = []
    for l in lines:
	s = l.split(",")
	label = s[0]
	loc = [s[1],s[2],s[3]]
	dim = [s[4],s[5],s[6]]
	hue = s[7]
	aff = s[8][1:-1].split(";")
	prop = s[8][1:-1].split(";")
	o = TableObject(label,loc,dim,hue,aff,prop)
	objects.append(o)
    return objects

class RetrievalState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded'],
                         input_keys=['goalIn'],
                         output_keys=['sourceCase'])
    if not _initialized:
      initGlobals()

  def execute(self, userdata):
    retrieval = Retrieval()
    case = retrieval.loadCase()
    userdata.sourceCase = case
    return 'succeeded'

