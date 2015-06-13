#!/usr/bin/env python
# Software License Agreement (BSD License)


# import time
# import rospy


class PIDController:
    def __init__(self, kp, kd, ki, dt, minLimit, maxLimit,nValue, normalizationFactor):
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
        print("Started PIDController with %s %s %s %s" %(kp,kd,ki,dt))


    def SetError(self, error):
        self.LastError = self.error
        self.error = error

    def GetFix(self):
        err = self.error
        lastError = self.LastError
        if abs(err)<self.epsilon:
            return self.nValue
        # Test it, it might be helpful, if not comment it
        if err > 10:
            self.integral=0
            print "error greater than 10, maxlimit set"
            return self.MaxLimit +self.normalizationFactor

        if err > 3:
            self.integral=0
            print "fix is 1400, error >3 but <10"
            return 1400

        if err < -10:
            self.integral=0
            print "fix is 1200, error greater than -10"
            return 1200

        if err < -3:
            self.integral=0
            print "fix is 1200, error greater than -3 but less than -10"
            return self.MinLimit+self.normalizationFactor

        print "Error between -10 and 10, pid working"
        #Comment until here if not needed
        self.integral += self.ki*err
        if self.integral > self.MaxLimit:
            self.integral = self.MaxLimit
        elif self.integral < self.MinLimit:
            self.integral = self.MinLimit
        output = self.kp*err + self.integral + self.kd*(err-lastError)/self.dt
        if output > self.MaxLimit:
            output = self.MaxLimit
        elif output < self.MinLimit:
            output = self.MinLimit
        output = output+self.normalizationFactor
        print("Integral: %s, Diverative %s, fix: %s" % (self.integral, self.kd*(self.error-self.LastError)/self.dt, output))
        return output

