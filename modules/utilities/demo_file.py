#!/bin/env python
import roslib; 
import rospy

class DemonstrationFile:
  def __init__(self, filename):
    self.filename = filename

  def loadFile(self):
    demo = []
    with open(self.filename, 'r') as f:
	line = f.readline()
	parsed = line.split(',')
	demo.append(parsed)
    f.close()
    return demo

  def writeFile(self, demo):
    f = open(self.filename, 'w')
    for line in demo:
	f.write(line)
    f.close()
