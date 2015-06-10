#!/usr/bin/env python
# Software License Agreement (BSD License)


# import time
# import rospy


class PIDController:
    def __init__(self, kp, kd, ki, dt, minLimit, maxLimit):
        self.kp = float(kp)
        self.kd = float(kd)
        self.ki = float(ki)
        self.dt = float(dt)
        self.integral = 0
        self.MaxLimit = float(maxLimit)
        self.MinLimit = float(minLimit)
        self.LastError = 0
        self.error=0
        print("Started PIDController with %s %s %s %s" %(kp,kd,ki,dt))


    def SetError(self, error):
        self.LastError = self.error
        self.error = error
        #print("Last error: %s, new error: %s" %(self.LastError, self.error))

    def GetFix(self):
        # Test it, it might be helpful, if not comment it
        if(self.error>10):
            self.integral=0
            print "fix is 1600 error>10"
            return 1600

        if(self.error>3):
            self.integral=0
            print "fix is 1400, error >3 but <10"
            return 1400

        if(self.error<-10):
            self.integral=0
            print "fix is 1100, error greater than -10"
            return 1200

        if(self.error<-3):
            self.integral=0
            print "fix is 1200, error greater than -3 but less than -10"
            return 1100

        print "Error between -10 and 10, pid working"
        #Comment until here if not needed
        self.integral += self.ki*self.error
        if self.integral > self.MaxLimit:
            self.integral = self.MaxLimit
        elif self.integral < self.MinLimit:
            self.integral = self.MinLimit
        output = self.kp*self.error + self.integral + self.kd*(self.error-self.LastError)/self.dt
        if output > self.MaxLimit:
            output = self.MaxLimit
        elif output < self.MinLimit:
            output = self.MinLimit
        print("Integral: %s, Diverative %s, fix: %s" % (self.integral, self.kd*(self.error-self.LastError)/self.dt, output+1000))
        return output + 1000

