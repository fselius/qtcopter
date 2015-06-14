#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Quaternion
from qtcopter.msg import controller_msg, uav_msg
from PIDModule import PIDController
import Configuration

#TODO: set this path relativly to any working directory (odroid, any dev machine)
config = Configuration.Configuration('/home/odroid/catkin_ws/src/qtcopter/src/qtcopter/src/qtcopter/navigation/PidConfig.json')
#config = Configuration.Configuration('/home/noam/catkin_ws/src/qtcopter/src/qtcopter/src/qtcopter/navigation/PidConfig.json')

class PIDManager:
    def __init__(self, dt, normalizationFactor):
        #Private members init
        xConfig = config.GetConfigurationSection("X")
        yConfig = config.GetConfigurationSection("Y")
        zConfig = config.GetConfigurationSection("Z")
        thetaConfig = config.GetConfigurationSection("Theta")
        self.AxisControllers = {
            "X": PIDController(xConfig["KP"], xConfig["KD"], xConfig["KI"], dt, xConfig["MinLimit"], xConfig["MaxLimit"], xConfig["NValue"], normalizationFactor, "X"),
            "Y": PIDController(yConfig["KP"], yConfig["KD"], yConfig["KI"], dt, yConfig["MinLimit"], yConfig["MaxLimit"], yConfig["NValue"], normalizationFactor, "Y"),
            "Z": PIDController(zConfig["KP"], zConfig["KD"], zConfig["KI"], dt, zConfig["MinLimit"], zConfig["MaxLimit"], zConfig["NValue"], normalizationFactor, "Z"),
            "Theta": PIDController(thetaConfig["KP"], thetaConfig["KD"], thetaConfig["KI"], dt, thetaConfig["MinLimit"], thetaConfig["MaxLimit"],thetaConfig["NValue"], normalizationFactor, "T")
            }
        self.IsRunning = False
        self.dt=dt
        self.normalizationFactor = normalizationFactor
        self.message = controller_msg()

        #Topics & Services init
        rospy.Subscriber("/pid_input", controller_msg, self.DataCollector)
        self.PidControlService = rospy.Service('/pid_control', Bool, self.PIDControlServiceRequestHandler)
        self.UpdateAxisConstantsService = rospy.Service('/pid_set_axis_gains', Quaternion, self.updateAxisGains)
        self.PidOutputTopic = rospy.Publisher('/pid/controller_command', controller_msg,queue_size=1)

    #Callback for updating error on each axis from /pid_input topic
    def DataCollector(self, msg):
        self.AxisControllers["X"].SetError(msg.x)
        self.AxisControllers["Y"].SetError(msg.y)
        self.AxisControllers["Z"].SetError(msg.z)
        self.AxisControllers["Theta"].SetError(msg.t)

    #Run gets called every #Rate and publish a fix if IsRunning set to true
    #Otherwise nothing happens until next check up
    def Run(self):
        if self.IsRunning:
            self.message.x = self.AxisControllers["X"].GetFix()
            self.message.z = self.AxisControllers["Z"].GetFix()
            self.message.y = self.AxisControllers["Y"].GetFix()
            self.message.t = self.AxisControllers["Theta"].GetFix()
            self.PidOutputTopic.publish(self.message)

    #Service for start/stop publishing on PID
    def PIDControlServiceRequestHandler(self, state):
        if not self.IsRunning == state:
            self.IsRunning = state

    #Service for updating gains on a specific axis
    #x = KP, y=KD, z=KI, w = axis name (X, Y , Z or Theta)
    #Would stop publish if is running to switch to new controller
    def updateAxisGains(self, gains):
        minLimit = self.AxisControllers[gains.w].MinLimit
        maxLimit = self.AxisControllers[gains.w].MaxLimit
        nValue = self.AxisControllers[gains.w].nValue
        controller = PIDController(gains.x,gains.y, gains.z,self.dt,minLimit,maxLimit, nValue,self.normalizationFactor, gains.w)
        isRunning = self.IsRunning
        if isRunning:
            self.IsRunning = False
        self.AxisControllers[gains.w] = controller
        self.IsRunning = isRunning

if __name__ == '__main__':
    try:
        rospy.init_node('pid_node')
        generalConfig = config.GetConfigurationSection("General")
        pidManager = PIDManager(generalConfig["Rate"], generalConfig["NormalizationFactor"])
        rate = rospy.Rate((int)(generalConfig["Rate"]))
        while not rospy.is_shutdown():
            pidManager.Run()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
