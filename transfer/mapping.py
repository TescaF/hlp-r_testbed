#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import sys
import time
import scipy.stats

def initGlobals():
  global _initialized

  _initialized = True

class Mapping:
  def __init__(self):

  def pruneMappingHypotheses(self, hypSpace, srcId, tgtId):

  def pruneFeatureSpace(self, ftSetSpace, srcId, tgtId):

  def evalHypotheses(self, hypSpace, ftSetSpace, src, tgt):
    e = np.zeros((len(src), len(tgt)))
    for i in range(len(src)):
      for j in range(len(tgt)):
	h = sim.hueSim(src[i].hue, tgt[j].hue)
	ch = sim.hueDiffSim(src[i].hue, tgt[j].hue, hueShift)
	d = sim.sizeSim(src[i].size, tgt[j].size)
	dh = sim.sizeDiffSim(src[i].size, tgt[j].size, sizeShift)
	s = sim.spatialSim(src.spatialRelns(i), tgt.spatialRelns(j))
	a = sim.affSim(src[i].affordances, tgt[j].affordances)
	p = sim.propSim(src[i].properties, tgt[j].properties)
	e[i][j] = [h, ch, d, dh, s, a, p]
    return e

  def predictMapping(self, hypSpace, ftSetSpace, evals):


class SimilarityMetric:

  def diff(self, v1, v2, r):
    dist = scipy.stats.norm(v1, 0.24 * r)
    n1 = dist.pdf(v2)
    n2 = dist.pdf(v1)
    return n1/n2

  def hueSim(self, h1, h2):
    v1 = 0
    v2 = (math.sin(h1-h2)/(math.cos(h1-h2))
    v2 = math.atan(v2)
    return diff(v1, v2, 360)

  def hueDiffSim(self, c1, c2, mean):
    v1 = c2
    v2 = c1 + mean
    return diff(v1, v2, 360)

  def sizeSim(self, d1, d2):
    v1 = 1
    v2 = (abs(d2-d1)/d1) + 1
    return diff(v1, v2, 1)

  def sizeDiffSim(self, d1, d2, mean):
    v1 = 0
    v2 = (abs(d2-d1)/d1) - mean
    return diff(v1, v2, 1)

  def spatialSim(self, s1, s2):
    v1 = len(s1)
    v2 = len([filter(lambda x: x in s1, sublist) for sublist in s2])
    return diff(v1, v2, len(s1))

  def affSim(self, a1, a2):
    v1 = len(a1)
    v2 = len([filter(lambda x: x in a1, sublist) for sublist in a2])
    return diff(v1, v2, min(v1, len(a2)))

  def propSim(self, p1, p2):
    v1 = len(p1)
    v2 = len([filter(lambda x: x in p1, sublist) for sublist in p2])
    return diff(v1, v2, min(v1, len(p2)))

class MappingState(smach.State):
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

