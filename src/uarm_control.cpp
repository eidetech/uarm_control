#include <cstdio>
#include <signal.h>
#include<unistd.h>
#include "uarm.h"
// #include "xarm_api/xarm_driver.h"
// #include "xarm_planner/xarm_planner.h"

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_driver_node] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_driver_node", node_options);

    RCLCPP_INFO(node->get_logger(), "namespace: %s", node->get_namespace());

    std::string robot_ip = "192.168.8.167";
    node->get_parameter_or("robot_ip", robot_ip, robot_ip);
    if (robot_ip == "") {
        RCLCPP_ERROR(node->get_logger(), "No param named 'robot_ip'");
        exit(1);
    }

    RCLCPP_INFO(node->get_logger(), "xarm_driver_node start");
    
    Uarm uarm(node, robot_ip);
    // rclcpp::Time t = node->now();
    // double seconds = t.seconds();

    // std::cout << seconds/1000000000 << " sec" << std::endl;

    // rclcpp::rate::RateBase::sleep() = node.create_timer(1);

    uarm.stop_gripper();

    // Pickup box from ground
    uarm.set_pos_home();
    uarm.set_pos_lookout();

    uarm.open_gripper();
    uarm.rotate_neg_90();
    uarm.set_pos_pickup();
    uarm.close_gripper();
    uarm.close_gripper();
    uarm.rotate_0();

    // // Move box to robot platform
    // uarm.set_pos_lookout();

    // uarm.set_pos_front_left();
    // uarm.set_pos_back_left();
    // uarm.set_pos_back();

    // // Unload box
    // // uarm.rotate_pos_90();
    // uarm.set_pos_unload();
    // uarm.open_gripper();
    // uarm.open_gripper();
    // uarm.set_pos_back();
    // // uarm.rotate_0();
    // // uarm.stop_gripper();

    // // Pick box back up
    // uarm.set_pos_unload();
    // uarm.close_gripper();
    // uarm.close_gripper();
    // uarm.close_gripper();
    // uarm.close_gripper();

    // uarm.set_pos_back();

    // // Move box back to ground
    // uarm.set_pos_back_left();
    // uarm.set_pos_front_left();
    // uarm.set_pos_lookout();
    // uarm.set_pos_home();

    // uarm.set_pos_pickup();
    // uarm.open_gripper();

    // // Go home and stop gripper
    // uarm.set_pos_lookout();
    // uarm.stop_gripper();
    // uarm.set_pos_home();


    // float vel = 400;
    // float acc = 50;
    // int del = 100;

    // Home
    // float pose[] = {100, 0, 300, 3.14, 0, 0};
    // xarm_driver.arm->set_position_aa(pose, -1, vel, acc, 0, false, -1, false, 0);

    // xarm_driver.arm->open_lite6_gripper();
    // sleep_milliseconds(1000);
    // xarm_driver.arm->close_lite6_gripper();
    // sleep_milliseconds(1000);
    // xarm_driver.arm->stop_lite6_gripper();

    // float pose[] = {270, 0, 60, 3.14, 0, 0};
    // xarm_driver.arm->set_position_aa(pose, -1, vel, acc, 0, false, -1, false, 0);
    
    // // Get current angles of all joints
    // float get_angles[7];
    // xarm_driver.arm->get_servo_angle(&get_angles[0]);

    // std::cout << get_angles[0] << " " << get_angles[1] << " " << get_angles[2] << " " << get_angles[3] << " " << get_angles[4] << " " << get_angles[5] << " " << get_angles[6] << " " << std::endl;

    // Insert current angles to all joints except end effector
    // float angles[7] = {get_angles[0], get_angles[1], get_angles[2], get_angles[3], get_angles[4], 1, get_angles[5]};
    // xarm_driver.arm->set_servo_angle(angles, vel, 1, 10, 0);
    // float pose[] = {-350, 0, 300, 3.14, 0, 0};
    // xarm_driver.arm->set_position_aa(pose, -1, vel, acc, 0, false, -1, false, 0);
    // float pose2[] = {-250, -250, 300, 3.14, 0, 0};
    // xarm_driver.arm->set_position_aa(pose2, -1, vel, acc, 0, false, -1, false, 0);
    // float pose3[] = {0, -250, 300, 3.14, 0, 0};
    // xarm_driver.arm->set_position_aa(pose3, -1, vel, acc, 0, false, -1, false, 0);
    // float pose4[] = {250, -250, 300, 3.14, 0, 0};
    // xarm_driver.arm->set_position_aa(pose4, -1, vel, acc, 0, false, -1, false, 0);
    // float pose6[] = {250, -250, 300, 3.14, 0, 0};
    // xarm_driver.arm->set_position_aa(pose6, -1, vel, acc, 0, false, -1, false, 0);
    // float pose7[] = {0, -250, 300, 3.14, 0, 0};
    // xarm_driver.arm->set_position_aa(pose7, -1, vel, acc, 0, false, -1, false, 0);
    // float pose8[] = {-250, -250, 300, 3.14, 0, 0};
    // xarm_driver.arm->set_position_aa(pose8, -1, vel, acc, 0, false, -1, false, 0);
    // float pose9[] = {-350, 0, 300, 3.14, 0, 0};
    // xarm_driver.arm->set_position_aa(pose9, -1, vel, acc, 0, false, -1, false, 0);
 
    signal(SIGINT, exit_sig_handler);
    rclcpp::spin(node);
    rclcpp::shutdown();

    RCLCPP_INFO(node->get_logger(), "xarm_driver_node over");

    return 0;
}