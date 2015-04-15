#!/usr/bin/env python
# Software License Agreement (BSD License)

import time
import rospy
from mavros.msg import OverrideRCIn
from mavros.srv import CommandBool
from mavros.srv import SetMode
from RcMessage import RcMessage
from mavros.msg import State

#FlightMode class : encapsulates all mode related operations of the drone
#To be further implemented

class FlightMode:
    __rcMessage = None
    __armingService = None
    __setModeService = None
    __rcOverrideTopic = None

    #Register to all needed topics/services for this object
    #init a RcMessage as a member for this class, to be able to control whats published
    #on the rc/override topic'''
    def __init__(self):
        self.__setModeService = rospy.ServiceProxy('/mavros/set_mode',SetMode)
        self.__armingService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.__rcOverrideTopic = rospy.Publisher('/mavros/rc/override',OverrideRCIn,queue_size = 10) #TBD : how to determine queue size
        self.__rcMessage = RcMessage()

    #Arm: arm/disarm the drone
    #param : armDisarmBool - true for arm, false for disarm
    #return value : success/failure
    def Arm(self, armDisarmBool):
        self.__rcMessage.ResetRcChannels()
        self.__rcMessage.SetThrottle(1000) #Set throttle to low speed so we can arm
        self.__rcOverrideTopic.publish(self.__rcMessage.GetRcMessage())
        time.sleep(1) #TBD : is this necessary? need to check if topic was grabbed
        try:
            return self.__armingService(armDisarmBool)
        except rospy.ServiceException as ex:
            print("Service did not process request: " + str(ex))
            return False

    #GetCurrentMode : get the current mode of flight as mode object
    #return value : State.mode object
    def GetCurrentMode(self):
        return State(rospy.wait_for_message('/mavros/state', State, timeout=1)).mode

    #SetCurrentMode : set the current mode of flight
    #param : mode - the requested mode of flight
    #return value : success/failure
    def SetCurrentMode(self, mode):
        return self.__setModeService(base_mode=0, custom_mode=mode)







