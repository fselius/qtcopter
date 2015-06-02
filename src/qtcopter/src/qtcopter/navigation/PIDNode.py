#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
from qtcopter.msg import controller_msg, uav_msg
from PIDModule import PIDController
import Configuration


config = Configuration.Configuration('/home/efishtain/PycharmProjects/qtcopter/src/qtcopter/src/qtcopter/navigation/PidConfig.json')

class PIDManager:
    def __init__(self, dt, minLimit, maxLimit):
        xConfig = config.GetConfigurationSection("X")
        yConfig = config.GetConfigurationSection("Y")
        zConfig = config.GetConfigurationSection("Z")
        thetaConfig = config.GetConfigurationSection("Theta")
        self.AxisControllers = {"X": PIDController(xConfig["KP"], xConfig["KD"], xConfig["KI"], dt, minLimit, maxLimit),
                                "Y": PIDController(yConfig["KP"], yConfig["KD"], yConfig["KI"], dt, minLimit, maxLimit),
                                "Z": PIDController(zConfig["KP"], zConfig["KD"], zConfig["KI"], dt, minLimit, maxLimit),
                                "Theta": PIDController(thetaConfig["KP"], thetaConfig["KI"], thetaConfig["KD"], dt, minLimit, maxLimit)
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
        pidManager = PIDManager(generalConfig["Rate"], generalConfig["MinLimit"], generalConfig["MaxLimit"])
        rate = rospy.Rate((int)(generalConfig["Rate"]))
        while not rospy.is_shutdown():
            fix = pidManager.Run()
            rate.sleep()
            pid_publisher.publish(fix)
    except rospy.ROSInterruptException:
        pass
