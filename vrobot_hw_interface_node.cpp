#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <rosserial_arduino/Test.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

class VRobotHWInterface : public hardware_interface::RobotHW {
public:
    VRobotHWInterface() {
        // Enregistrer les joints dans l'interface matérielle
        for (int i = 0; i < 5; ++i) {
            hardware_interface::JointStateHandle state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
            jnt_state_interface.registerHandle(state_handle);
            hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_names[i]), &cmd[i]);
            jnt_pos_interface.registerHandle(pos_handle);
        }
        registerInterface(&jnt_state_interface);
        registerInterface(&jnt_pos_interface);
    }

    void read() {
        // Logique pour lire les positions actuelles des joints
        // Par exemple, depuis Arduino via rosserial
    }

    void write() {
        // Logique pour envoyer les commandes des joints à Arduino
        // Par exemple, via rosserial
    }

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;

    double cmd[5];  // Commandes
    double pos[5];  // Positions
    double vel[5];  // Vitesses
    double eff[5];  // Efforts
    std::string joint_names[5] = {"joint_0", "joint_1", "joint_2", "joint_3", "joint_11"};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "vrobot_hw_interface");
    ros::NodeHandle nh;

    VRobotHWInterface robot_hw;
    controller_manager::ControllerManager cm(&robot_hw, nh);

    ros::Rate rate(50);  // Fréquence de mise à jour
    while (ros::ok()) {
        robot_hw.read();
        cm.update(ros::Time::now(), ros::Duration(0.02));
        robot_hw.write();
        rate.sleep();
    }
    return 0;
}

