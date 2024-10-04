#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from hardware_interface import RobotHW
from controller_manager_msgs.srv import SwitchController

class VRobotHWInterface(RobotHW):
    def __init__(self):
        rospy.init_node('vrobot_hw_interface', anonymous=True)
        self.joint_names = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_11"]
        self.cmd = [0] * len(self.joint_names)
        self.pos = [0] * len(self.joint_names)
        self.vel = [0] * len(self.joint_names)
        self.eff = [0] * len(self.joint_names)

        # Publisher pour envoyer les commandes aux joints
        self.joint_pubs = [rospy.Publisher(f"/{joint_name}_position_controller/command", Float64, queue_size=10)
                           for joint_name in self.joint_names]

        # Assure-toi que le service de gestion des contrôleurs est disponible
        rospy.wait_for_service('/controller_manager/switch_controller')
        self.switch_controller_srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

    def read(self):
        # Logique pour lire les positions actuelles des joints, à implémenter selon ton setup matériel
        pass

    def write(self):
        # Logique pour envoyer les commandes des joints
        for i, pub in enumerate(self.joint_pubs):
            pub.publish(Float64(self.cmd[i]))

    def control_loop(self):
        rate = rospy.Rate(50)  # 50 Hz de fréquence
        while not rospy.is_shutdown():
            self.read()
            self.write()
            rate.sleep()

if __name__ == '__main__':
    robot_hw_interface = VRobotHWInterface()
    robot_hw_interface.control_loop()

