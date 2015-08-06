#!/usr/bin/env python
# Software License Agreement (BSD License)

# import time
# import rospy

#===========================================================================================
#PIDController class
#Class is responsible over 1 PID
#an instance is created with provided gains, min/max limits and normalization factor
#main responsibility of the instance is to hold provided errors and
#calculte the needed fix depending on provided gains
#===========================================================================================
class PIDController:
    def __init__(self, kp, kd, ki, dt, minLimit, maxLimit,nValue, normalizationFactor, axis):
        #Private members init
        #Gains
        self.kp = float(kp)
        self.kd = float(kd)
        self.ki = float(ki)

        #Delta time, used to calculate derivative
        self.dt = float(dt)

        #integral part of PID
        self.integral = 0

        #Min/Max limits for the output
        self.MaxLimit = float(maxLimit)
        self.MinLimit = float(minLimit)

        #Used to hold lastError and new error
        #used for derivative calculations and fixes
        self.LastError = 0
        self.error = 0

        #used to normalize factor
        self.normalizationFactor = float(normalizationFactor)
        self.nValue = nValue

        #used for float diff comparison
        self.epsilon = 0.01

        #axis name
        self.axis = axis
        print("Started PIDController with %s %s %s %s" %(kp,kd,ki,dt))

    #======================================================================
    #SetError method
    #Saves current error to LastError
    #update error to the new error recived
    #======================================================================
    def SetError(self, error):
        self.LastError = self.error
        self.error = error

    #======================================================================
    #GetFix method
    #Main logic of class, used to calculate the needed fix based on:
    #   error, lastError, gains, integral, derivative and linear fix
    #======================================================================
    def GetFix(self):
        #assign values to local variables so change while in loop won't hurt
        #calculations. Cross threading handling.
        err = self.error
        lastError = self.LastError

        # #if err is less than epsilon, no fix is needed
        # #the nominal value is returned
        # #1500 for roll,pitch,yaw and aroun 1300 for throttle
        # if abs(err)<self.epsilon:
        #     return self.nValue #maybe need to return here NO_CHANGE from RCMessageOverride constant

        #Error Ranges in meters
        #Divide err to ranges: Greated than 10, Between 3 and 10, Greater then -10 (error to different
        #side of the axis) and between -3 and -10

        #if err is greater than 10 meters
        #Reset the integral component of the PID
        #fix is maximum available fix as defined in configuration
        if err > 10:
            self.integral=0
            print self.axis, ":error greater than 10, maxlimit set"
            return self.MaxLimit +self.normalizationFactor

        #if err is between 3 and 10 meters
        #Reset the integral component of the PID
        #change state to "Between" fix meaning medimum rate fix
        if err > 3:
            self.integral=0
            print self.axis, ":fix is 1400, error >3 but <10"
            return 1400

        #if err is greater than -10 meters
        #Reset the integral component of the PID
        #fix is minimum available fix as defined in configuration
        if err < -10:
            self.integral=0
            print self.axis, "error greater than -10, MinLimit set"
            return self.MinLimit+self.normalizationFactor

        #if err is between 3 and 10 meters
        #Reset the integral component of the PID
        #change state to "Between" fix meaning medimum rate fix
        if err < -3:
            self.integral=0
            print self.axis, ":fix is 1200, error greater <-3 but >-10"
            return 1200

        #Error is between -3 and 3
        #Fix is calculated to be err*kp + integral*ki + derivative*kd
        #integral component is bounded in order to prevent windup effect
        #output in bounded in order not to hurt quadcopter
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

