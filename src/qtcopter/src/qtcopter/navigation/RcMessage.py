#!/usr/bin/env python
# Software License Agreement (BSD License)

from mavros.msg import OverrideRCIn
import inspect

class ChannelName:
    Roll, Pitch, Throttle, Yaw = range(4)
    HumanOverrideChannel = 7

class RcMessage:
    __MINIMUM_VAL = 1000
    __MID_VAL = 1500
    __MAXIMUM_VAL = 2000
    __THROTTLE_ARM_VALUE = 1000
    __rcChannels = None

    def __init__(self):
        self.__rcChannels = OverrideRCIn().channels

    #GetRcMessage : packs a msg object and returns it to caller
    #return value : OverrideRCIn msg with channels set
    def GetRcMessage(self):
        msg = OverrideRCIn()
        msg.channels = self.__rcChannels
        return msg

    def ResetRcChannels(self):
        self.__PrintDebugMessage("Reseting RC channels input")
        for i in range(0,7):
            self.__rcChannels[i] = OverrideRCIn.CHAN_RELEASE


    #set all rc_channels with mid value
    def BalanceRcChannels(self):
        for i in range(0,3):
            self.__rcChannels[i] = self.__MID_VAL

    def PrepareForArming(self):
        self.BalanceRcChannels()
        self.SetThrottle(self.__THROTTLE_ARM_VALUE)
        self.SetHumanOverride(OverrideRCIn.CHAN_NOCHANGE)

    def SetRoll(self, value):
        self.__rcChannels[ChannelName.Roll] = value

    def SetPitch(self, value):
        self.__rcChannels[ChannelName.Pitch] = value

    def SetThrottle(self, value):
        self.__rcChannels[ChannelName.Throttle] = value

    def SetYaw(self, value):
        self.__rcChannels[ChannelName.Yaw] = value

    def SetHumanOverride(self, value):
        self.__rcChannels[ChannelName.HumanOverrideChannel] = value

    def GetRoll(self):
        return self.__rcChannels[ChannelName.Roll]

    def GetPitch(self):
        return self.__rcChannels[ChannelName.Pitch]

    def GetThrottle(self):
        return self.__rcChannels[ChannelName.Throttle]

    def GetRoll(self):
        return self.__rcChannels[ChannelName.Yaw]

    def GetHumanOverride(self):
        return self.__rcChannels[ChannelName.HumanOverrideChannel]

    def __PrintDebugMessage(self, msg):
        class_name = self.__class__.__name__
        current_frame = inspect.currentframe()
        func = inspect.getframeinfo(current_frame.f_back).function
        print str("{0}.{1}():".format(class_name,func))
        print str(" {0}".format(msg))




