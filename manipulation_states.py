#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

import sys
import time

from geometry_msgs.msg import Pose

from speech_synth import speech_synthesizer 
from speech_recog import speech_listener
from manipulation import manipulator
from manipulation.manipulator import *
from manipulation.arm_moveit import *
from modules.manipulation_module import DMPLearner

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
                [-1.70, 2.00, 1.00, -2.20, 2.00, 0.90],
                [-1.60, 2.20, 0.80, -2.20, 1.50, 1.20],
                [-1.60, 2.40, 1.00, -2.50, 1.50, 1.20],
                [-1.60, 2.60, 1.20, -3.14, 1.50, 1.20]]

  eeTra =      [[0.40, 0, 0.50, 0.0, 0.0, 0.0]]

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
      self.arm.sendWaypointTrajectory(_traDict['person'])
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

class ExecuteEETrajectoryState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['succeeded','failed','aborted'],
                         input_keys=['trajectoryIn','timingsIn'],
                         output_keys=['execResultOut'])


    if not _isTheManipulationStateGlobalsInitialized:
      initGlobals()
    self.arm = _manipulator.arm # Arm()
    self.arm_planner = _arm_planner
    self.ss = _speech_synth #speech_synthesizer.SpeechSynthesizer()

    self.functionDict = {'retract':self.arm.upper_tuck}

  def resetPose(self):
    print 'going to reset position'
    sendPlanRevised(self.arm, _traDict['reset'])
    time.sleep(3.0)

  def planTrajectory(self, points):
    self.arm_planner.group[0].set_pose_reference_frame('base_link')
    trajectories = []
    for pt in points:
      pose = Pose()
      pose.position.x = pt[0]
      pose.position.y = pt[1]
      pose.position.z = pt[2]
      pose.orientation = [pt[3],pt[4],pt[5]]
      newTra = self.arm_planner.plan_poseTargetInput(pose)
      if newTra is None:
	return None
      trajectories.append(newTra)
    return trajectories

  def execute(self, userdata):
    self.resetPose()
    rospy.loginfo('Moving the arm')
    userdata.execResultOut = None
    if userdata.trajectoryIn is None:
      print 'Executing saved trajectory'
      #self.arm.sendWaypointTrajectory(_traDict['person'])
      traj = self.planTrajectory(_traDict['ee'])
      if traj is None:
	print 'Could not find a plan'
	return 'failed'
      print traj
      sendPlanList(self.arm, traj)
      return 'succeeded'
    else:
      print 'Executing trajectory'
      sendTimedPlan(self.arm, userdata.trajectoryIn, userdata.timingsIn)
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
    goal2 = [-1.60, 0, 1.20, -3.14, 1.50, 1.20]
    goal_thresh = [0.2,0.2,0.2,0.2,0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * req.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 1       #dt is rather large, so this is > 1  
    plan = self.learner.makePlanRequest(x_0, x_dot_0, t_0, goal2, goal_thresh,
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
    goal1 = [-1.60, 3.14, 1.20, 0.0, 0.0, 0.0]
    goal_thresh = [0.2,0.2,0.2,0.2,0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * req.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 1       #dt is rather large, so this is > 1  
    plan = self.learner.makePlanRequest(x_0, x_dot_0, t_0, goal2, goal_thresh,
                           seg_length, tau, dt, integrate_iter)

    self.ss.say("done!")
    time.sleep(0.5)
    converted,timings = self.convertPlan(plan)
    userdata.traOut = converted 
    userdata.timings = timings
    time.sleep(1.5)
    return 'succeeded'
