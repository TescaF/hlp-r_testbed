#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import sys
import time
import rosbag
import action_utils.ActionUtils

def initGlobals():
  global _initialized

  _initialized = True

class Learning:
  def __init__(self, directory, name):
    self.directory = directory
    self.name = name
    self.utils = ActionUtils()
    self.dmps = None

  def getTrajectories(self):
    fileNum = 0
    filename = self.directory + self.name + str(fileNum) + ".bag"
    trajectories = []
    durations = []
    while os.path.isfile(filename):
      self.bag = rosbag.Bag(filename)
      traj = []
      minTime = None

      #read trajectory from bag file
      for topic, msg, t in self.bag.read_messages(topics=['eef_pose']):
        pos = msg.data.position
        orient = msg.data.orientation
        traj.append([pos.x,pos.y,pos.z,orient.x,orient,y,orient.z,orient.w])
        if minTime is None:
	  minTime = t
        maxTime = t
      trajectories.append(traj)
      duration.append(maxTime - minTime)

      fileNum += 1
      filename = self.directory + self.name + str(fileNum) + ".bag"

    return trajectories, durations 

  def learnDMPs(self, trajectories, durations):
    dmps = []
    i = 0
    for traj in trajectories:
      dmps.append(self.utils.learnDMP(traj, durations(i)))
      i += 1
    return dmps

 #   while not self.server.is_preempt_requested():
 #     if goal.eef_only == True:

  def deriveTargeters(self, trajectories, objects):
    targeters = []
    for t in trajectories:
      endPt = t[-1]
      minDist = -1
      minObj = None
      for o in objects:
	d = dist(o, endPt)
	if minObj is None or d < minDist:
	  minDist = d
	  minObj = o
      exp = []
      exp.append(endPt[0] - minObj.location[0])
      exp.append(endPt[1] - minObj.location[1])
      exp.append(endPt[2] - minObj.location[2])
      expr = ""
      for e in exp:
	expr += "(" + str() + ")"
	if e < 0:
	  expr += "-"
	else:
	  expr += "+"
	expr += abs(e) + ","
      targeters.append(expr[:-1]
    return targeters

class CaseWriter:
  def __init__(self, directory, dmps, targeters, objects):
    writeDMPs(directory + "dmps.txt", dmps)
    writeTargeters(directory + "targeters.txt", targeters)
    writeObjects(directory + "objects.txt", objects)

  def writeDMPs(self, filename, dmps):
    f = open(filename, 'w')
    for dmp in dmps:
      f.write("k_gain:" + str(dmp.k_gain) + '\n')
      f.write("d_gain:" + str(dmp.d_gain) + '\n')

      wStr = "weights:"
      for w in dmp.weights:
	wStr += str(w) + ','
      f.write(wStr[:-1] + '\n')

      dStr = "f_domain:"
      for d in dmp.f_domain:
	dStr += str(d) + ','
      f.write(dStr[:-1] + '\n')

      tStr = "f_targets:"
      for t in dmp.f_targets:
	tStr += str(t) + ','
      f.write(wStr[:-1] + '\n')
    f.close()
      
  def writeTargeters(self, filename, targeters):
    f = open(filename, 'w')
    for t in targeters:
      f.write(t + '\n')
    f.close()

  def writeObjects(self, filename, objects):
    f = open(filename, 'w')
    for o in objects:
      oStr = str(o.location[0]) + "," + str(o.location[1]) + "," + str(o.location[2]) + "," + str(o.dimensions[0]) + "," + str(o.dimensions[1]) + "," + str(o.dimensions[2]) + "," + str(o.hue) + ",{"
      for aff in o.affordances:
        oStr += aff + ";"
      oStr = oStr[:-1] + "},{"
      for prop in o.properties:
        oStr += prop + ";"
      oStr = oStr[:-1] + "}\n"
      f.write(oStr)
    f.close()

class LearningState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded'],
                         input_keys=['directoryIn, objectsIn'],
                         output_keys=['statusOut'])
    if not _initialized:
      initGlobals()

  def execute(self, userdata):
    learner = Learning()
    trajectories, durations = learner.getTrajectories()
    dmps = learner.learnDMPs(trajectories, durations)
    targeters = learner.deriveTargeters(trajectories, userdata.objectsIn)
    CaseWriter(userdata.directoryIn, dmps, targeters, userdata.objectsIn)
    userdata.statusOut = 'succeeded'
    return 'succeeded'

