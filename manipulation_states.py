#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import tf
import sys
import time

from geometry_msgs.msg import Pose

from speech_synth import speech_synthesizer 
from speech_recog import speech_listener
from manipulation import manipulator
from manipulation.manipulator import *
from manipulation.arm_moveit import *
from modules.manipulation_module import DMPLearner
from trac_ik_wrapper.srv import *
from modules.utilities.baris_utils import *

_manipulator = None
_arm_planner = None
_speech_synth = None
_isTheManipulationStateGlobalsInitialized = False

#see if it makes sense to pass these with userdata
def initGlobals():
  print 'Initializing manipulation state globals'

  global _manipulator
  global _arm_planner
  global _speech_synth
  global _isTheManipulationStateGlobalsInitialized
  global _traDict

  _arm_planner = ArmMoveIt()
  _manipulator = Manipulator()
  _speech_synth = speech_synthesizer.SpeechSynthesizer()
  _isTheManipulationStateGlobalsInitialized = True

  handOffTra = [[-1.90, 1.50, 0.50, -2.00, 3.00, 0.72],
                [-1.80, 1.80, 1.00, -2.10, 2.50, 0.72],
                [-1.70, 1.90, 1.00, -2.20, 2.00, 0.90],
                [-1.60, 2.00, 0.80, -2.20, 1.50, 1.20],
                [-1.60, 2.10, 1.00, -2.50, 1.50, 1.20],
                [-1.60, 2.20, 1.20, -3.14, 1.50, 1.20]]

#  eeTra = 	[[0.087, 0.167, 0.169, 0.175, 0.954, 0.127, 0.207],
#		[0.181, 0.183, 0.417, 0.239, 0.844, 0.214, 0.429], 
#		[0.241, 0.133, 0.542, 0.299, 0.664, 0.380, 0.570], 
#		[0.244, -0.014, 0.555, 0.455, 0.421, 0.426, 0.659], 
#		[0.241, -0.0572, 0.685, 0.379, 0.329, 0.507, 0.701], 
#		[0.169, -0.103, 0.859, 0.191, 0.111, 0.639, 0.737]]
  eeTra =	[[0.554,0.039,1.180,0.235,0.799,0.551,0.057],
		[0.714,0.094,1.183,0.473,0.786,0.391,0.073],
		[0.835,0.132,1.149,0.667,0.684,0.264,0.134],
		[0.820,0.141,1.047,0.766,0.622,-0.012,0.162],
		[0.916,0.149,0.998,0.851,0.505,-0.034,0.138],
		[1.046,0.126,0.946,0.970,0.225,-0.056,0.072]]

  resetPos = [[-1.90, 1.50, 0.50, -2.00, 3.00, 0.72]]

  _traDict = {'person':handOffTra,'reset':resetPos,'ee':eeTra}

def sendPlan(arm,plannedTra):
  traj_goal = FollowJointTrajectoryGoal()

  traj_goal.trajectory = plannedTra.joint_trajectory
  arm.smooth_joint_trajectory_client.send_goal(traj_goal)#sendWaypointTrajectory(traj_goal)
  arm.smooth_joint_trajectory_client.wait_for_result()   
  return arm.smooth_joint_trajectory_client.get_result() 

def sendPlanRevised(arm, plannedTra):
  for pt in plannedTra:
    print "goto: " + str(pt) 
    arm.ang_pos_cmd(pt)
 #   arm.lin_cmd(0.5)
    time.sleep(1.0)
  return

def sendPlanList(arm, plannedTra):
  for traj in plannedTra:
    print "goto: " + str(traj[-1]) 
    for pt in traj:
      arm.ang_pos_cmd(pt)
    time.sleep(1.0)
  return


def sendTimedPlan(arm, plannedTra, timings):
  startTime = time.clock()
  i = 0
  for pt in plannedTra:
    goalTime = timings[i] + startTime
    #while time.clock() < goalTime:
    #  time.sleep(0.1)
    i = i+1
    print "goto: " + str(pt) 
    arm.ang_pos_cmd(pt)
  return

class ExecuteTrajectoryState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=['trajectoryIn','timingsIn'],
                         output_keys=['execResultOut'])


    if not _isTheManipulationStateGlobalsInitialized:
      initGlobals()
    self.arm = _manipulator.arm # Arm()
    self.ss = _speech_synth #speech_synthesizer.SpeechSynthesizer()

    self.functionDict = {'retract':self.arm.upper_tuck}

  def resetPose(self):
    print 'going to reset position'
    sendPlanRevised(self.arm, _traDict['reset'])
    time.sleep(3.0)

  def execute(self, userdata):
    self.resetPose()
    rospy.loginfo('Moving the arm')
    userdata.execResultOut = None
    if userdata.trajectoryIn is None:
      print 'Executing saved trajectory'
 #     sendPlan(self.arm, self.traDict['person'])  
    #  self.arm.sendWaypointTrajectory(_traDict['person'])
      sendPlanRevised(self.arm, _traDict['person'])
      return 'succeeded'
    print 'Executing trajectory'
    sendTimedPlan(self.arm, userdata.trajectoryIn, userdata.timingsIn)
    self.ss.say("Now executing the arm trajectory")
    #if not isinstance(userdata.trajectoryIn, str):
    #  if isinstance(userdata.trajectoryIn, list):
    #print userdata.trajectoryIn
    #self.arm.sendWaypointTrajectory(userdata.trajectoryIn)
    #  else:
    #    sendPlan(self.arm, userdata.trajectoryIn)  
    userdata.execResultOut = 'done'
    return 'succeeded'

class PlanDMPState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=[],
                         output_keys=['traOut','timings'])

    if not _isTheManipulationStateGlobalsInitialized:
      initGlobals()

    self.arm_planner = _arm_planner
    self.ss = _speech_synth #speech_synthesizer.SpeechSynthesizer()
    self.learner = DMPLearner(6,100,4)
    self.targetPose = None

  def convertPlan(self, original):
    pts = original.plan.points
    plan = []
    timings = original.plan.times
    for pt in pts:
      pos = pt.positions
      plan.append(pos)
    return plan,timings

  def execute(self, userdata):
    rospy.loginfo('Calculating path')
    userdata.traOut = None

    dt = 1.0
    req = self.learner.makeLFDRequest(_traDict['person'], dt)
    self.learner.makeSetActiveRequest(req.dmp_list)

    x_0 = [-1.90, 1.50, 0.50, -2.00, 3.00, 0.72]
    x_dot_0 = [0.0,0.0,0.0,0.0,0.0,0.0]
    t_0 = 0
    #goal = [8.0,7.0]         #Plan to a different goal than demo
    goal1 = [-1.60, 3.14, 1.20, -3.14, 1.50, 1.20]
    goal2 = [2.60, 3.14, 1.20, -3.14, 1.50, 1.20]
                #[-1.60, 2.60, 1.20, -3.14, 1.50, 1.20]]
 #   goal2 = [3.1, 0, 1.20, -3.14, 1.50, 1.20]
    goal_thresh = [0.2,0.2,0.2,0.2,0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * req.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 1       #dt is rather large, so this is > 1  
    plan = self.learner.makePlanRequest(x_0, x_dot_0, t_0, goal1, goal_thresh,
                           seg_length, tau, dt, integrate_iter)

    self.ss.say("done!")
    time.sleep(0.5)
    converted,timings = self.convertPlan(plan)
    userdata.traOut = converted 
    userdata.timings = timings
    time.sleep(1.5)
    return 'succeeded'

class PlanEEDMPState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=[],
                         output_keys=['traOut','timings'])

    if not _isTheManipulationStateGlobalsInitialized:
      initGlobals()

    self.arm_planner = _arm_planner
    self.ss = _speech_synth #speech_synthesizer.SpeechSynthesizer()
    self.learner = DMPLearner(6,100,4)
    self.targetPose = None
    self.tf = tf.TransformListener(True)
 
  def convertPlan(self, original):
    pts = original.plan.points
    plan = []
    timings = original.plan.times
    for pt in pts:
      pos = pt.positions
      plan.append(pos)
    return plan,timings

  def getIK(self, seed, goal):
    try:
      ik = rospy.ServiceProxy('trac_ik_wrapper', IKHandler)
      response = ik(seed, goal)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
      return None
    return response.pose

  def transform(self, pt, originFrame, targetFrame):
    ptMsg = PoseStamped()
    ptMsg.header.stamp = rospy.Time.now()
    ptMsg.header.frame_id = originFrame
    ptMsg.pose.position.x = pt[0]
    ptMsg.pose.position.y = pt[1]
    ptMsg.pose.position.z = pt[2]
    if len(pt) is 6:
      quat = tf.transformations.quaternion_from_euler(pt[3],pt[4],pt[5])
    if len(pt) is 7:
      quat = (pt[3],pt[4],pt[5],pt[6])
    ptMsg.pose.orientation.x = quat[0]
    ptMsg.pose.orientation.y = quat[1]
    ptMsg.pose.orientation.z = quat[2]
    ptMsg.pose.orientation.w = quat[3]
    self.tf.waitForTransform(originFrame, targetFrame, ptMsg.header.stamp, rospy.Duration(5.0))
    tfPoint = self.tf.transformPose(targetFrame, ptMsg)
    newQuat = (tfPoint.pose.orientation.x,tfPoint.pose.orientation.y,tfPoint.pose.orientation.z,tfPoint.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(newQuat)
    return [tfPoint.pose.position.x, tfPoint.pose.position.y, tfPoint.pose.position.z,
	euler[0], euler[1], euler[2]]

  def execute(self, userdata):
    rospy.loginfo('Calculating path')
    userdata.traOut = None

    dt = 1.0
    req = self.learner.makeLFDRequest(_traDict['person'], dt)
    self.learner.makeSetActiveRequest(req.dmp_list)

    x_0 = _traDict['ee'][0] # [-1.90, 1.50, 0.50, -2.00, 3.00, 0.72]
    x_dot_0 = [0.0,0.0,0.0,0.0,0.0,0.0]
    t_0 = 0
    #goal = [8.0,7.0]         #Plan to a different goal than demo
    #goal1 = [0.554,0.038,1.1298,0.2354,0.7983,0.551,0.565] #_traDict['ee'][5]
    #goal2 = [1.046,0.126,0.946,0.970,0.225,-0.056,0.072]
    goal1 = [1.046,0.126,1.2,0.970,0.225,-0.056,0.072]
    goal2 = [0.98,0.26,1.2,0.970,0.225,-0.056,0.072]
    #goal2 = [0.98,0.06,1.25,0.970,0.225,-0.056,0.072]
    goal_thresh = [0.2,0.2,0.2,0.2,0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * req.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 1       #dt is rather large, so this is > 1  
    
    js_x_0 = self.getIK(_traDict['person'][0], self.transform(x_0,"/base_link","/linear_actuator_link"))
    js_goal = self.getIK(_traDict['person'][5], self.transform(goal2,"/base_link","/linear_actuator_link"))
    if (js_x_0 is None) or (js_goal is None):
	return 'failed'
    plan = self.learner.makePlanRequest(js_x_0, x_dot_0, t_0, js_goal, goal_thresh,
                           seg_length, tau, dt, integrate_iter)

    self.ss.say("done!")
    time.sleep(0.5)
    converted,timings = self.convertPlan(plan)
    userdata.traOut = converted 
    userdata.timings = timings
    time.sleep(1.5)
    return 'succeeded'
