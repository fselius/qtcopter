#!/usr/bin/env python
# Software License Agreement (BSD License)


# import time
# import rospy


class PIDController:
    def __init__(self, kp, kd, ki, dt, minLimit, maxLimit,nValue, normalizationFactor, axis):
        self.kp = float(kp)
        self.kd = float(kd)
        self.ki = float(ki)
        self.dt = float(dt)
        self.integral = 0
        self.MaxLimit = float(maxLimit)
        self.MinLimit = float(minLimit)
        self.LastError = 0
        self.error = 0
        self.normalizationFactor = normalizationFactor
        self.nValue = nValue
        self.epsilon = 0.01
        self.axis = axis
        print("Started PIDController with %s %s %s %s" %(kp,kd,ki,dt))


    def SetError(self, error):
        self.LastError = self.error
        self.error = error

    def GetFix(self):
        err = self.error
        lastError = self.LastError
        if abs(err)<self.epsilon:
            return self.nValue #maybe need to return here NO_CHANGE from RCMessageOverride constant

        if err > 10:
            self.integral=0
            print self.axis, ":error greater than 10, maxlimit set"
            return self.MaxLimit +self.normalizationFactor

        if err > 3:
            self.integral=0
            print self.axis, ":fix is 1400, error >3 but <10"
            return 1400

        if err < -10:
            self.integral=0
            print self.axis, ":fix is 1200, error greater than -10"
            return 1200

        if err < -3:
            self.integral=0
            print self.axis, "error greater than -3, MinLimit set"
            return self.MinLimit+self.normalizationFactor

        self.integral += self.ki*err
        if self.integral > self.MaxLimit:
            self.integral = self.MaxLimit
        elif self.integral < self.MinLimit:
            self.integral = self.MinLimit
        derivative = self.kd*(err-lastError)/self.dt
        output = self.kp*err + self.integral + derivative
        if output > self.MaxLimit:
            output = self.MaxLimit
        elif output < self.MinLimit:
            output = self.MinLimit
        output = output+self.normalizationFactor
        print self.axis, ":Error %s, Proportinal %s, Integral %s, Derivative %s, Fix %s" %(err, err*self.kp, self.integral, derivative, output)
        return output

