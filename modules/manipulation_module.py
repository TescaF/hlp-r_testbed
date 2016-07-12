#!/usr/bin/env python
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *

class DMPLearner:
	def __init__(self,dims=2,k_gains=100,num_bases=4):
	    self.dims = dims
	    self.k_gains = k_gains
	    self.d_gains = 2.0 * np.sqrt(k_gains)
	    self.num_bases = num_bases
	
	#Learn a DMP from demonstration data
	def makeLFDRequest(self,traj, dt): 
	    demotraj = DMPTraj()
		
	    for i in range(len(traj)):
		pt = DMPPoint();
		pt.positions = traj[i]
		demotraj.points.append(pt)
		demotraj.times.append(dt*i)
		    
	    k_gains = [self.k_gains]*self.dims
	    d_gains = [self.d_gains]*self.dims
		
	    print "Starting LfD..."
	    rospy.wait_for_service('learn_dmp_from_demo')
	    try:
		lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
		resp = lfd(demotraj, k_gains, d_gains, self.num_bases)
	    except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	    print "LfD done"    
	    return resp;


	#Set a DMP as active for planning
	def makeSetActiveRequest(self, dmp_list):
	    try:
		sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
		sad(dmp_list)
	    except rospy.ServiceException, e:
		print "Service call failed: %s"%e


	#Generate a plan from a DMP
  	def makePlanRequest(self, x_0, x_dot_0, t_0, goal, goal_thresh, 
			    seg_length, tau, dt, integrate_iter):
  	    print "Starting DMP planning..."
	    rospy.wait_for_service('get_dmp_plan')
	    try:
		gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
		resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
		   seg_length, tau, dt, integrate_iter)
	    except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	    print "DMP planning done"   
	    return resp;


if __name__ == '__main__':
    rospy.init_node('dmp_tutorial_node')

    #Create a DMP from a 2-D trajectory
    dt = 1.0                
    learner = dmpLearner(2, 100, 4)
    traj = [[1.0,1.0],[2.0,2.0],[3.0,4.0],[6.0,8.0]]
    resp = learner.makeLFDRequest(traj, dt)

    #Set it as the active DMP
    learner.makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    x_0 = [0.0,0.0]          #Plan starting at a different point than demo 
    x_dot_0 = [0.0,0.0]   
    t_0 = 0                
    goal = [8.0,7.0]         #Plan to a different goal than demo
    goal_thresh = [0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * resp.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1  
    plan = learner.makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)

    print plan

