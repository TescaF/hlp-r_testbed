#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import tf
import signal
import sys
import time

from geometry_msgs.msg import Pose
from decimal import *
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

def initGlobals():
  print 'Initializing manipulation state globals'

  global _manipulator
  global _arm_planner
  global _speech_synth
  global _isTheManipulationStateGlobalsInitialized
  global _traDict
  global _ptNum
  global _group

  _ptNum = 0
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

  eeTra =      [[0.554,0.039,1.580,0.235,0.799,0.551],
		[0.765,0.091,1.598,0.455,0.766,0.428],
		[0.826,0.129,1.573,0.663,0.669,0.289],
		[0.812,0.139,1.493,0.768,0.597,0.052],
		[0.915,0.146,1.502,0.852,0.471,0.092],
		[1.047,0.119,1.549,0.966,0.190,0.137]]

  resetPos = [[-1.90, 1.50, 0.50, -2.00, 3.00, 0.72]]

  _traDict = {'person':handOffTra,'reset':resetPos, 'ee':eeTra}

  moveit_commander.roscpp_initialize(sys.argv)
  _robot = moveit_commander.RobotCommander()
  _scene = moveit_commander.PlanningSceneInterface()
  _group = [moveit_commander.MoveGroupCommander("arm"), moveit_commander.MoveGroupCommander("gripper"),
         moveit_commander.MoveGroupCommander("head")]

def sigint_handler(signum, frame):
  sys.exit(1)

signal.signal(signal.SIGINT, sigint_handler)

def sendPlan(arm, plannedTra):
  for pt in plannedTra:
    arm.ang_pos_cmd(pt)
    #time.sleep(0.1)
  return

def sendPlanTrajectory(arm, plannedTra):
  for pt in plannedTra:
    arm.ang_pos_cmd(pt.positions)
    time.sleep(0.1)
  return

def sendPlanNew(arm, plannedTra):
  newPlan = []
  for pt in plannedTra:
    print "goto: " + str(pt) 
    #try:
    _group[0].set_joint_value_target(pt)
    subplan = _group[0].plan().joint_trajectory.points
    for pt2 in subplan:
      newPlan.append(pt2.positions)
    #except Exception:
    #  print "Point out-of-bounds"
  for pt in newPlan:
    arm.ang_pos_cmd(pt)
    #time.sleep(1.0)
  return

def sendEEPlan (arm, plannedTra):
  waypoints = []
  for pt in plannedTra:
    _group[0].set_position_target(pt)
    subplan = _group[0].plan().joint_trajectory.points
    for pt2 in subplan:
      waypoints.append(pt2.positions)
  for pt in waypoints:
    arm.ang_pos_cmd(pt)
    time.sleep(0.1)

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
    self.ptNum = -1 
    self.functionDict = {'retract':self.arm.upper_tuck}

  def resetPose(self):
    print 'going to reset position'
    sendPlan(self.arm, _traDict['reset'])
    return

  def execute(self, userdata):
    self.resetPose()
    rospy.loginfo('Moving the arm')
    userdata.execResultOut = None
    if userdata.trajectoryIn is None:
      print 'Executing saved trajectory'
      sendPlan(self.arm, _traDict['person'])
      return 'succeeded'
    elif userdata.trajectoryIn is 'reset':
      return 'succeeded'
    else:
      print 'Executing trajectory'
      sendPlanNew(self.arm, userdata.trajectoryIn)
      #sendEEPlan(self.arm, userdata.trajectoryIn)
      self.ss.say("Now executing the arm trajectory")
      userdata.execResultOut = 'done'
      return 'succeeded'

class PlanEEDMPState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=['goal'],
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

  def getIK(self, ptCount, seed, goals):
    try:
      ik = rospy.ServiceProxy('trac_ik_wrapper', IKHandler)
      response = ik(ptCount, seed, goals)
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
    print userdata.goal
    self.goal = [userdata.goal.translation.x, userdata.goal.translation.y, userdata.goal.translation.z,-3.14, 1.50, 1.20]
    print self.goal

    dt = 1.0
    req = self.learner.makeLFDRequest(_traDict['ee'], dt)
    self.learner.makeSetActiveRequest(req.dmp_list)

    x_0 = [0.554,0.039,1.180,0.235,0.799,0.551]
    x_dot_0 = [0.0,0.0,0.0,0.0,0.0,0.0]
    t_0 = 0
    goal1 = [0.97,0.093,1.32,-0.51,0.51,0.51]
    goal2 = [1.25,-0.2199,1.499,-0.51,0.51,0.51]
    goal_thresh = [0.2,0.2,0.2,0.2,0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * req.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 1  
    
    ee_x_0 = self.transform(x_0,"/base_link","/linear_actuator_link")
    js_x_0 = self.getIK(1,_traDict['person'][0], self.transform(x_0,"/base_link","/linear_actuator_link"))
    ee_goal = self.transform(self.goal,"/base_link","/linear_actuator_link")
    plan = self.learner.makePlanRequest(ee_x_0, x_dot_0, t_0, ee_goal, goal_thresh,
               seg_length, tau, dt, integrate_iter)
    cvt_plan,timings = self.convertPlan(plan)
    #cvt_plan = _traDict['ee']
    print 'ee plan'
    for pt in cvt_plan:
      print pt

    #userdata.traOut = cvt_plan
    #return 'succeeded'

    js_plan = []
    prev = _traDict['person'][0] #js_x_0
    print 'js plan'
    seeds = []
    goals = []
    for pt in cvt_plan:
     # tr_pt = self.transform(pt, "/base_link","/linear_actuator_link")
      p = [pt[0],pt[1],pt[2],pt[3],pt[4],pt[5]]
      goals = goals + p
      
    js_pts = self.getIK(len(cvt_plan), prev, goals)
    for i in range (0,len(js_pts),6):
        js_pt = [js_pts[i],js_pts[i+1],js_pts[i+2],js_pts[i+3],js_pts[i+4],js_pts[i+5]]
        print js_pt
	js_plan.append(js_pt)
 #   prev = js_pt
 #   js_plan.append(js_pt)
    userdata.traOut = js_plan
    #userdata.timings = timings
    userdata.timings = None
    return 'succeeded'
