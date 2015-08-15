#!/usr/bin/env python
# Software License Agreement (BSD License)

import time
import rospy
from mavros.msg import OverrideRCIn, RCIn
from mavros.srv import SetMode, StreamRate, CommandBool, StreamRateRequest
from threading import Thread
from RcMessage import RcMessage
from Configuration import Configuration
from qtcopter.msg import controller_msg
from qtcopter.srv import *
import inspect

config = Configuration("NavConfig.json")

class HumanOverride:
    def __init__(self):
        self.ChangeToMode = None
        self.Flag = False

class Navigator:
    __rcMessage = None
    __armingService = None
    __setModeService = None
    __rcOverrideTopic = None
    __humanOverrideDefault = None
    __humanOverrideElapsedTime = 0
    __navigatorParams = None
    __baseGlobalPosition = None
    __currentGlobalPosition = None
    __currentMode = None

    #Register to all needed topics/services for this object
    #init a RcMessage as a member for this class, to be able to control whats published
    #on the rc/override topic
    def __init__(self):
        self.__navigatorParams = config.GetConfigurationSection("params")
        self.__humanOverrideDefault = self.__navigatorParams["HumanOverrideDefault"]
        self.__setModeService = rospy.Service('navigator/set_mode', NavigatorModeSrv, self.__SetCurrentMode)
        self.__armingService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.__setModeMavros = rospy.ServiceProxy('/mavros/set_mode',SetMode)
        self.__pidControlService = rospy.ServiceProxy('/pid_control', PidControlSrv)
        self.__rcOverrideTopic = rospy.Publisher('/mavros/rc/override',OverrideRCIn,queue_size = 10) #TBD : how to determine queue size
        self.__rcInListener = rospy.Subscriber('/mavros/rc/in',RCIn,self.__HumanOverrideCallback)
        self.__rcMessage = RcMessage()
        self.__humanOverride = HumanOverride()
        self.__humanOverride.Flag = False
        self.__isPidRunning = True
        self.__numOfPublishThreads = 0
        self.__isArmed = False
        self.__isRcReseted = False
        self.__rcMessage.ResetRcChannels()
        time.sleep(1) # because ROS is so fucked up
        self.__rcOverrideTopic.publish(self.__rcMessage.GetRcMessage())
        rospy.loginfo("Finished initializing Navigator node.")

    #Arm: arm/disarm the drone
    #param : armDisarmBool - true for arm, false for disarm
    #return value : success/failure
    def Arm(self, armDisarmBool):
        self.__baseGlobalPosition = self.__currentGlobalPosition
        self.__rcMessage.PrepareForArming()
        self.__rcOverrideTopic.publish(self.__rcMessage.GetRcMessage())
        time.sleep(1) #TBD : is this necessary? need to check if topic was grabbed
        try:
            self.__armingService(armDisarmBool)
            self.__isArmed = True
        except rospy.ServiceException as ex:
            rospy.logerr("Service did not process request: " + str(ex))
            return False

    #ConstantRatePublish : publishing the outputs of pid_controller to rc_override channels
    #at 25hz rate (defined in configuration file).
    #runs as a separate thread so the set_mode service won't be occupied.
    def __ConstantRatePublish(self, arg):
        if self.__numOfPublishThreads > 0:
            rospy.logerr("A publish thread is already running, cannot init another thread. Aborting.")
            return 1
        self.__numOfPublishThreads += 1
        rate = rospy.Rate(self.__navigatorParams["PublishRate"])
        while self.__currentMode.upper() == 'PID_ACTIVE' or self.__currentMode.upper() == 'PID_ACTIVE_HOLD_ALT':
            if not self.__humanOverride.Flag and self.__isPidRunning:
                stime = time.time()
                try:
                    msg = rospy.wait_for_message('/pid/controller_command',controller_msg)
                    self.PublishRCMessage(msg.x, msg.y, msg.z, msg.t)
                    rate.sleep()
                    etime = time.time()
                    str = "publishing : {0} {1} {2} {3} elapsed:{4}".format(msg.x,msg.y,msg.z,msg.t,etime - stime)
                    rospy.loginfo(str)
                except:
                    rospy.logerr("controller msg from pid topic did not process")
                    break
            else:
                rospy.loginfo("Human override activated, publishing thread stopping...")
                break
        self.__numOfPublishThreads = 0
        return 1

    #SetCurrentMode : set the current mode of flight (navigator/set_mode callback)
    #activated by calling 'navigator/set_mode' service with with mode and additional param
    #params : mode - the requested mode of flight as SetMode
    #return value : success/failure
    def __SetCurrentMode(self, request):

        self.__currentMode = request.message
        var = request.message.upper()
        if var == 'ARM':
            self.__setModeMavros(base_mode=0, custom_mode='STABILIZE')
            self.Arm(True)
        elif var == 'DISARM':
            self.__setModeMavros(base_mode=0, custom_mode='STABILIZE')
            self.Arm(False)
        elif var == 'ALT_HOLD':
            self.__setModeMavros(base_mode=0, custom_mode='ALT_HOLD')
        elif var == 'POSHOLD' or var == 'POS_HOLD':
            res = self.__setModeMavros(base_mode=0, custom_mode='POSHOLD')
        elif var == 'LAND':
            self.__setModeMavros(base_mode=0, custom_mode='LAND')
            self.__isArmed = False
        elif var == 'STABILIZE':
            self.__setModeMavros(base_mode=0, custom_mode='STABILIZE')
        elif var == 'PID_ACTIVE':
            self.__pidControlService(True)
            self.__isPidRunning=True
            thread = Thread(target = self.__ConstantRatePublish, args = (0, ))
            thread.start()
        elif var == 'PID_ACTIVE_HOLD_ALT':
            self.__setModeMavros(base_mode=0, custom_mode='ALT_HOLD')
            thread = Thread(target = self.__ConstantRatePublish, args = (0, ))
            thread.start()
        elif var == 'PID_STOP':
            self.__isPidRunning=False
            self.__pidControlService(False)

        rospy.loginfo("Navigator is changing flight mode | mode: " + self.__currentMode.upper())
        return true

    #PublishRCMessage : publish new message to rc/override topic to set rc channels
    #params : roll, pitch, throttle, yaw as integer values between 1000 to 2000
    def PublishRCMessage(self, roll, pitch, throttle, yaw):
        if self.__IsPublishAllowed():
            self.__rcMessage.SetRoll(roll)
            self.__rcMessage.SetPitch(pitch)
            self.__rcMessage.SetThrottle(throttle)
            self.__rcMessage.SetYaw(yaw)
            self.__rcOverrideTopic.publish(self.__rcMessage.GetRcMessage())

    #HumanOverrideCallback : Constantly checking rc/override HumanOverride channel and maintaining a
    #boolean flag according to that
    def __HumanOverrideCallback(self, data):
        if not self.__isRcReseted:
            self.__humanOverrideElapsedTime = time.time()
            val = data.channels[self.__navigatorParams["HumanOverrideChannel"]]
            threshHold = self.__navigatorParams["HumanOverrideThreshold"]

            #if rc8 switch in the controller will be turned down -> mode will change to POS_HOLD/ALT_HOLD
            #if rc8 switch in the controller will be turned up -> mode will change to LAND
            if (val < self.__humanOverrideDefault - threshHold):
                self.__humanOverride.Flag = True
                self.__humanOverride.ChangeToMode = self.__navigatorParams["HumanOverrideModeChange"]
                self.__AfterHumanOverridePublish()
            elif (val > self.__humanOverrideDefault + threshHold):
                self.__humanOverride.Flag = True
                self.__humanOverride.ChangeToMode = 'LAND'
                self.__AfterHumanOverridePublish()

    #Performs as reset to all rc channels to bring control back to the operator
    def __AfterHumanOverridePublish(self):
        self.__isRcReseted = True
        self.__rcMessage.PrepareForArming() #(pitch,roll,yaw)<-1500, throttle<-1000
        self.__rcOverrideTopic.publish(self.__rcMessage.GetRcMessage())
        self.__rcMessage.ResetRcChannels() #release all channels to allow human full control
        self.__rcOverrideTopic.publish(self.__rcMessage.GetRcMessage())
        self.__setModeMavros(base_mode=0, custom_mode=str(self.__humanOverride.ChangeToMode))
        rospy.loginfo("Mode changed to {0}".format(self.__humanOverride.ChangeToMode))

    #IsPublishAllowed : Make all safety checks in this method.
    #return value : True/False according to all safety checks.
    def __IsPublishAllowed(self):
        if self.__humanOverride.Flag or self.__humanOverrideElapsedTime != 0 and \
                (time.time() - self.__humanOverrideElapsedTime > self.__navigatorParams["HumanOverrideElapsedTimeAllowed"]):
            rospy.loginfo("Human override channel activated, publish disabled.")
            rospy.loginfo("Navigator is changing to mode {0}.".format(self.__humanOverride.ChangeToMode))
            return False
        else:
            return True

if __name__ == '__main__':
    try:
        rospy.init_node('navigator', anonymous=True)
        nav = Navigator()
        #while not rospy.is_shutdown():
            #time.sleep(0)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass







