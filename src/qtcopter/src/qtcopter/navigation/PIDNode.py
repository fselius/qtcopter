#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
from qtcopter.msg import controller_msg, uav_msg
from PIDModule import PIDController
import Configuration

#TODO: set this path relativly to any working directory (odroid, any dev machine)
config = Configuration.Configuration('/home/odroid/catkin_ws/src/qtcopter/src/qtcopter/src/qtcopter/navigation/PidConfig.json')
#config = Configuration.Configuration('/home/noam/catkin_ws/src/qtcopter/src/qtcopter/src/qtcopter/navigation/PidConfig.json')

class PIDManager:
    def __init__(self, dt, normalizationFactor):
        xConfig = config.GetConfigurationSection("X")
        yConfig = config.GetConfigurationSection("Y")
        zConfig = config.GetConfigurationSection("Z")
        thetaConfig = config.GetConfigurationSection("Theta")
        self.AxisControllers = {"X": PIDController(xConfig["KP"], xConfig["KD"], xConfig["KI"], dt, xConfig["MinLimit"], xConfig["MaxLimit"],xConfig["NValue"], normalizationFactor),
                                "Y": PIDController(yConfig["KP"], yConfig["KD"], yConfig["KI"], dt, yConfig["MinLimit"], yConfig["MaxLimit"],yConfig["NValue"], normalizationFactor),
                                "Z": PIDController(zConfig["KP"], zConfig["KD"], zConfig["KI"], dt, zConfig["MinLimit"],zConfig["MaxLimit"],zConfig["NValue"], normalizationFactor),
                                "Theta": PIDController(thetaConfig["KP"], thetaConfig["KI"], thetaConfig["KD"], dt, thetaConfig["MinLimit"],thetaConfig["MaxLimit"],thetaConfig["NValue"], normalizationFactor)
                                }
        rospy.Subscriber("/pid_input", controller_msg, self.DataCollector)

    def DataCollector(self, msg):
        self.AxisControllers["X"].SetError(msg.x)
        self.AxisControllers["Y"].SetError(msg.y)
        self.AxisControllers["Z"].SetError(msg.z)
        self.AxisControllers["Theta"].SetError(msg.t)

    def Run(self):
        msg = controller_msg()
        msg.x = self.AxisControllers["X"].GetFix()
        msg.z = self.AxisControllers["Z"].GetFix()
        msg.y = self.AxisControllers["Y"].GetFix()
        msg.t = self.AxisControllers["Theta"].GetFix()
        return msg

if __name__ == '__main__':
    try:
        rospy.init_node('pid_node')
        pid_publisher = rospy.Publisher('/pid/controller_command',controller_msg,queue_size=1)
        generalConfig = config.GetConfigurationSection("General")
        pidManager = PIDManager(generalConfig["Rate"], generalConfig["NormalizationFactor"])
        rate = rospy.Rate((int)(generalConfig["Rate"]))
        while not rospy.is_shutdown():
            fix = pidManager.Run()
            pid_publisher.publish(fix)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
