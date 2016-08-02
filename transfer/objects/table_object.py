#!/usr/bin/env python

class TableObject:
  def __init__(self, label, location, dimensions, hue, affordances, properties):
    self.featureList = ["h", "ch", "d", "dh", "s", "a", "p"]
    self.label = label
    self.location = locations
    self.dimensions = dimensions
    self.size = dimensions[0] * dimensions[1] * dimensions[2]
    self.hue = hue
    self.hueDiff = 0
    self.sizeDiff = 0
    self.spatialSet = None
    self.affordances = affordances
    self.properties = properties

  def setSpatialRelations(self, spatial):
    self.spatialSet = spatial

  def getFeatureValue(self, feature):
    if feature is "h":
      return self.hue
    if feature is "ch":
      return self.hueDiff
    if feature is "d":
      return self.size
    if feature is "dh":
      return self.sizeDiff
    if feature is "s":
      return self.spatialSet
    if feature is "a":
      return self.affordances
    if feature is "p":
      return self.properties
  
  def indexOfFeature(self, feature):
    for i in range(len(featureList)):
      if feature is featureList[i]:
	return i
    return -1 
  
  
  
  
  
