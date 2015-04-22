#!/usr/bin/env python
# Software License Agreement (BSD License)


# import time
# import rospy


class PIDController:
    def __init__(self, kp, kd, ki, dt, maxLimit, minLimit):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt
        self.integral = 0
        self.MaxLimit = maxLimit
        self.MinLimit = minLimit

    def SetError(self, error):
        self.error = error

    def GetFix(self):
        self.integral += self.ki*self.error
        if self.integral > self.MaxLimit:
            self.integral = self.MaxLimit
        elif self.integral < self.MinLimit:
            self.integral = self.MinLimit
        output = self.kp*self.error + self.integral + self.kd*self.error/self.dt
        if output > self.MaxLimit:
            output = self.MaxLimit
        elif output < self.MinLimit:
            output = self.MinLimit
        return output



# #PIDModule : A type for PID operations, all calculations logic are encapsulated
# #within this class
# class PIDModule:
#
#     def __init__(self, Kp, Kd, Ki, dt):
#         self.__pid_sum = 0
#         self.__Kp = Kp
#         self.__Kd = Kd
#         self.__Ki = Ki
#         self.__lastTime = time.time()
#         self.__dt = dt
#
#     def ProportionalMethod(self, delta_x):
#         return self.__Kp * delta_x
#
#     def DifferentialMethod(self, delta_x, delta_t):
#         try:
#             return self.__Kd * (delta_x / delta_t)
#         except ZeroDivisionError, e:
#             print e
#             #TODO: logger handling
#
#     def IntegralMethod(self, delta_x, delta_t):
#         print ""
#         #TODO: implement this
#
#     def GetPIDValue(self, delta):
#         elapsed = time.time() - self.__lastTime
#         self.__lastTime = time.time()
#         self.__pid_sum = 0
#         self.__pid_sum += self.ProportionalMethod(delta) + \
#             self.DiffrentialMethod(delta, elapsed) + self.IntegralMethod(delta, elapsed)
#         return self.__pid_sum