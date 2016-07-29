#!/usr/bin/env python

class Case:
  def __init__(self, location, name, dmps, targeters, objects):
    self.location = location
    self.name = name
    self.dmps = dmps
    self.targeters = targeters
    self.objects = objects

  def getObjectFromLabel(self, label):
    for o in self.objects:
	if o.label == label:
	  return o

  def evalTarget(self, skillNum):
    t = self.targeters[skillNum]

    splitT = t.split(",")
    i = 0
    target = []
    for subT in splitT:
	#find all object expressions
	splitP = subT.split("(")
	for subP in splitP:
	  exp = subP.split(")")[0]
	  subT.replace(exp, self.getObjectFromLabel(exp)[i])
	target.append(eval(subT))
	i += 1
    return target
