#!/usr/bin/env python
# Software License Agreement (BSD License)

import time
import rospy

#PIDModule : A type for PID operations, all calculations logic are encapsulated
#within this class
class PIDModule:

    def __init__(self, Kp, Kd, Ki):
        self.__pid_sum = 0
        self.__Kp = Kp
        self.__Kd = Kd
        self.__Ki = Ki
        self.__lastTime = time.time()

    def ProportionalMethod(self, delta_x):
        return self.__Kp * delta_x

    def DifferentialMethod(self, delta_x, delta_t):
        try:
            return self.__Kd * (delta_x / delta_t)
        except ZeroDivisionError, e:
            print e
            #TODO: logger handling

    def IntegralMethod(self, delta_x, delta_t):
        print ""
        #TODO: implement this

    def GetPIDValue(self, delta):
        elapsed = time.time() - self.__lastTime
        self.__lastTime = time.time()
        self.__pid_sum = 0
        self.__pid_sum += self.ProportionalMethod(delta) + \
            self.DiffrentialMethod(delta, elapsed) + self.IntegralMethod(delta, elapsed)

        return self.__pid_sum
