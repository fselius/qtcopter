#!/usr/bin/env python
# Software License Agreement (BSD License)

# import time
import rospy

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
        self.nValue = float(nValue)

        #used for float diff comparison
        self.epsilon = 0.01

        #axis name
        self.axis = axis
        rospy.loginfo("Started PIDController with {0} {1} {2}".format(kp,kd,ki,dt))

    #======================================================================
    #SetError method
    #Saves current error to LastError
    #update error to the new error recived
    #======================================================================
    def SetError(self, error):
        self.LastError = self.error
        self.error = error

    #======================================================================
    #ResetIntegral method
    #Resets the integral accumulator and sets errors counters to zero
    #Used after aerial switching of PID gains
    #======================================================================
    def ResetIntegral(self):
        self.integral = 0
        self.LastError = 0
        self.error = 0
        rospy.loginfo("Resetting integral for channel: {0}".format(self.axis))

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


        #Fix is calculated to be err*kp + integral*ki + derivative*kd
        #integral component is bounded in order to prevent windup effect
        #output in bounded in order not to hurt quadcopter
        self.integral += (self.ki*err)/self.dt
        if self.integral > self.MaxLimit:
            self.integral = self.MaxLimit
        elif self.integral < self.MinLimit:
            self.integral = self.MinLimit
        derivative = self.kd*(err-lastError)*self.dt
        output = self.kp*err + self.integral + derivative
        if output > self.MaxLimit:
            output = self.MaxLimit
        elif output < self.MinLimit:
            output = self.MinLimit
        output = output+self.nValue
        rospy.loginfo("{0} :Error {1}, P: {2}, I: {3}, D: {4}, Fix {5}".format(self.axis, err, err*self.kp, self.integral, derivative, output))
        return output

