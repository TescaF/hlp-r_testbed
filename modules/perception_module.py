import roslib; 
from std_msgs.msg import *
from geometry_msgs.msg import *
import rospy
import time

from utilities.kinect_pantilt import PanTilt
from utilities.transform_listener import TransformListener

from math import sqrt

class ObjectSearch:
  def __init__(self, withRobot=True, tr_root = 'base_link'):
    
    self.online = withRobot

    self.pantilt = PanTilt( self.online)
    self.tf = TransformListener(useTf=True,  withRobot=self.online, transform_root = tr_root)

    self.searchPattern = [[0.0,0.7], [0.3, 0.6],[-0.3, 0.6],[-0.3, 0.8],[0.3, 0.8]]
    self.pt_rate   = 20
    self.pt_repeat = 20

  def objectSearch(self):
    if not self.online:
      return True

    self.tf.updateTransform()
    objectFound = False
    for i in range(0, len(self.searchPattern)):
      self.pantilt.adjust(self.searchPattern[i], self.pt_rate, self.pt_repeat)
      time.sleep(0.5)
      if(self.tf.updateTransform()):
        print self.tf.tr
        if not(self.tf.isTrAllZero()):
          objectFound = True
          break
    return objectFound

  def directLookAtObject(self):
    if not self.online:
      return 
    self.tf.updateTransform()
    pos = [self.tf.tr.translation.x, self.tf.tr.translation.y, self.tf.tr.translation.z]
    posInBase = self.pantilt.baseToObject(pos)
    h_ik = self.pantilt.headIK(posInBase)
    self.pantilt.adjust([h_ik[0], h_ik[1]],3,3)
    print h_ik


if __name__ == '__main__':
  rospy.init_node('object_search')
  os = ObjectSearch()

  if(os.objectSearch()):
    os.directLookAtObject()
    os.tf.updateTransform()
  else:
    print 'no obj found'
    os.pantilt.adjust([0,0],10,10)

