#!/bin/env python
import roslib; 
from geometry_msgs.msg import *
import rospy
import tf2_ros
import time
from math import sqrt
from baris_utils import *

class TransformListener:
  def __init__(self, useTf = False, withRobot=True, transform_root = 'base_link'):
    self.online = withRobot
    if self.online:
      if not useTf:
        self.objPoseSub = rospy.Subscriber("/baris/objectTransform", Transform, self.cb)
        self.tr_obj = None
        self.use_tf = False
      else:
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.use_tf = useTf
        self.tr_obj = Transform()
    else:
      self.use_tf = False

    self.object_root = transform_root

    self.tr = Transform()

  def cb(self, inTransform):
    self.tr = inTransform

  def isTrAllZero(self):
    tmp = self.tr.translation.x + self.tr.translation.y + self.tr.translation.z + \
          self.tr.rotation.x + self.tr.rotation.y + self.tr.rotation.z + self.tr.rotation.w
    return tmp == 0

  def fillTransform(self,trans):
    self.tr.translation.x = trans_tuple[0]
    self.tr.translation.y = trans_tuple[1]
    self.tr.translation.z = trans_tuple[2]
    
    self.tr.rotation.x = rot_tuple[0]
    self.tr.rotation.y = rot_tuple[1]
    self.tr.rotation.z = rot_tuple[2]
    self.tr.rotation.w = rot_tuple[3]
    
  def getTransform(self):
    if self.use_tf:
      self.updateTransform()
    
    return self.tr

  def getObjectTransform(self):
    if not self.use_tf:
      return None

    self.updateTransform()
    
    return self.tr_obj

  def getAverageObjectTransform(self, window_size = 10, rate = 10):

    if not self.use_tf:
      return None

    rrate = rospy.Rate(rate)
    self.updateTransform()
    average = rosTransformCopy(self.tr_obj)
    for i in range(0, window_size):
      rrate.sleep()
      self.updateTransform()
      average = rosTransformAdd(average, self.tr_obj, False)#transformAddition(average, self.tr_obj)
    
    average = rosTransformScalarMult(average, 1./(window_size+1))
    average.rotation = quatNormalize(average.rotation, True)
    return average
    

  def updateTransform(self):
    if not self.online:
      return True
    if not self.use_tf:
      print 'Not using tf. Initialize this class with use_tf = True in order to use this function'
      return True
    else:
      try:
        trans = self.tfBuffer.lookup_transform(self.object_root, 'main_object', rospy.Time(0))
        self.tr_obj = trans.transform 
        trans = self.tfBuffer.lookup_transform('kinect_ir_optical_frame', 'main_object', rospy.Time(0))
        self.tr = trans.transform
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print 'Cannot lookupTransform'
        return False
    return True
