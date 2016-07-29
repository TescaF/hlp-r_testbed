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
from trajectory_msgs.msg import JointTrajectoryPoint
from decimal import *
from manipulation import manipulator
from manipulation.manipulator import *
from manipulation.arm_moveit import *
from modules.manipulation_module import DMPLearner
from trac_ik_wrapper.srv import *

class ActionUtils:
  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    manip = Manipulator()
    self.arm = manip.arm
    self.group = [moveit_commander.MoveGroupCommander("arm"), moveit_commander.MoveGroupCommander("gripper"),
         moveit_commander.MoveGroupCommander("head")]
    self.resetPose = [-1.90, 1.50, 0.50, -2.00, 3.00, 0.72]
    self.dmpLearner = None
    self.tf = tf.TransformListener(True)

  def sendPlan(plannedTra):
    for pt in plannedTra:
      self.arm.ang_pos_cmd(pt)
    return

  def sendMoveItPlan(plannedTra):
    newPlan = []
    for pt in plannedTra:
      print "goto: " + str(pt) 
      self.group[0].set_joint_value_target(pt)
      subplan = self.group[0].plan().joint_trajectory.points
      for pt2 in subplan:
        newPlan.append(pt2.positions)
    for pt in newPlan:
      self.arm.ang_pos_cmd(pt)
    return

  def resetPose(self):
    print 'going to reset position'
    self.sendPlan(self.arm, self.resetPose)
    return

  def convertDMPPlanToTraj(self, original):
    pts = original.plan.points
    plan = []
    timings = original.plan.times
    for pt in pts:
      pos = pt.positions
      plan.append(pos)
    return plan,timings

  def getIK(self, seed, goals, tol):
    try:
      ik = rospy.ServiceProxy('trac_ik_wrapper', IKHandler)
      response = ik(seed, goals, tol)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
      return None
    return response.poses

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

  def convertCoordsToPoses(self, goals):
    converted = []
    for g in goals:
      initPose = Pose()
      initPose.position.x = g[0]
      initPose.position.y = g[1]
      initPose.position.z = g[2]
      quat = tf.transformations.quaternion_from_euler(g[3],g[4],g[5])
      initPose.orientation.x = quat[0]
      initPose.orientation.y = quat[1]
      initPose.orientation.z = quat[2]
      initPose.orientation.w = quat[3]
      converted.append(initPose)
    return converted

  def learnDMP(self, demo, dt):
    if self.dmpLearner is None:
      self.dmpLearner = DMPLearner(6,100,4)
    req = self.dmpLearner.makeLFDRequest(demo, dt)
    return req.dmp_list

  def getDMPEETrajectory(self, dmps, x_0, goal):
    self.dmpLearner.makeSetActiveRequest(dmps)
    x_dot_0 = [0.0,0.0,0.0,0.0,0.0,0.0]
    t_0 = 0
    goal_thresh = [0.2,0.2,0.2,0.2,0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * req.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 1  
    plan = self.learner.makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
               seg_length, tau, dt, integrate_iter)
    cvt_plan,timings = self.convertDMPPlanToTraj(plan)

  def getDMPJointTrajectory(self, dmps, x_0, goal, tol):
    plan,timings = getDMPEETrajectory(dmps, x_0, goal)
    goalPoses = self.convertCoordsToPoses(plan)
    js_pts = self.getIK(prev,goalPoses,tol)
    for i in range (0,len(js_pts),6):
        js_pt = [js_pts[i].positions[0],js_pts[i].positions[1],js_pts[i].positions[2],js_pts[i].positions[3],js_pts[i].positions[4],js_pts[i].positions[5]]
	js_plan.append(js_pt)
    return js_plan
