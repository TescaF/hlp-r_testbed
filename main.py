#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

import sys
import time

from manipulation_states_simple import *
from perception_states import *
from speech_recog import speech_listener
from speech_synth import speech_synthesizer
from manipulation.manipulator import *

#comments:
#execute might take all the cycles, not very c6 like, see if smach has an updateOnce/executeOnce option

class IdleState(smach.State):
  def __init__(self):
    smach.State.__init__(self, 
                         outcomes=['idling','execute_original','execute_dmp','find_object','reset'],
                         input_keys=[],
                         output_keys=['defaultTrajectory'] )
    self.sl =  speech_listener.SpeechListener()
    self.ss = speech_synthesizer.SpeechSynthesizer()

  def execute(self, userdata):
    speech_com = self.sl.get_last_command()
    userdata.defaultTrajectory = 'reset' 

    if speech_com is not None:
      print speech_com

    time.sleep(0.5)
        
    if speech_com == 'START':
      self.ss.say("Here we go")
      return 'execute_original'
    if speech_com == 'RED_OBJ':
      return 'find_object'
    if speech_com == 'STOP':
      return 'reset'
    return 'idling'

    
def main():
  rospy.init_node('smach_example_state_machine')

  # Create a SMACH state machine
  sm = smach.StateMachine(outcomes=['end'])
  
  withR = False

  # Open the container
  with sm:
    # Add states to the container
    smach.StateMachine.add('Idle', IdleState(), 
                            transitions={'idling':'Idle', 
                                         'execute_original':'ExecuteTrajectory',
                                         'find_object':'FindObject',
					 'reset':'Reset',
                                         'execute_dmp':'PlanDMP'},
                            remapping={'defaultTrajectory':'reset'}) 

    smach.StateMachine.add('FindObject', FindObjectState(), 
                            transitions={'succeeded':'PlanDMP', 
                                         'failed':'Idle'},
                                         remapping={'objectLocationOut':'objectLocation'})

    smach.StateMachine.add('Reset', ExecuteTrajectoryState(), 
                            transitions={'succeeded':'Idle', 
                                         'failed':'Idle',
                                         'aborted':'Idle'},
                                         remapping={'trajectoryIn':'reset','timingsIn':'timings'})

    smach.StateMachine.add('ExecuteTrajectory', ExecuteTrajectoryState(), 
                            transitions={'succeeded':'Idle', 
                                         'failed':'Idle',
                                         'aborted':'Idle'},
                                         remapping={'trajectoryIn':'tra','timingsIn':'timings'})

    smach.StateMachine.add('PlanDMP', PlanEEDMPState(), 
                            transitions={'succeeded':'ExecuteTrajectory', 
                            #transitions={'succeeded':'Idle', 
                                         'failed':'Idle',
                                         'aborted':'Idle'},
                                         remapping={'traOut':'tra','timings':'timings','goal':'objectLocation'})
                                                    
  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()

  # Execute SMACH plan
  outcome = sm.execute()
  sis.stop()


if __name__ == '__main__':
  main()
