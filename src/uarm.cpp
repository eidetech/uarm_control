#include "uarm.h"

Uarm::Uarm(rclcpp::Node::SharedPtr& node, std::string &robot_ip)
{
    xarm_driver.init(node, robot_ip);

    xarm_driver.arm->motion_enable(1, 8);

    xarm_driver.arm->set_state(XARM_STATE::STOP);
    xarm_driver.arm->set_mode(0);
    sleep_milliseconds(10);
    xarm_driver.arm->set_state(0);
}

Uarm::~Uarm()
{
}

// Home pos
void Uarm::set_pos_home()
{
    float pose[] = {100, 0, 300, 3.14, 0, 0};
    xarm_driver.arm->set_position(pose, -1, _vel, _acc, 0, false, -1, false, 0);
    float get_angles[7];
    xarm_driver.arm->get_servo_angle(&get_angles[0]);
    std::cout << get_angles[0] << " " << get_angles[1] << " " << get_angles[2] << " " << get_angles[3] << " " << get_angles[4] << " " << get_angles[5] << " " << get_angles[6] << " " << std::endl;

}

// Lookout pos
void Uarm::set_pos_lookout()
{
    float pose[] = {350, 0, 300, 3.14, 0, 0};
    xarm_driver.arm->set_position(pose, -1, _vel, _acc, 0, false, -1, false, 0);
    float get_angles[7];
    xarm_driver.arm->get_servo_angle(&get_angles[0]);
    std::cout << get_angles[0] << " " << get_angles[1] << " " << get_angles[2] << " " << get_angles[3] << " " << get_angles[4] << " " << get_angles[5] << " " << get_angles[6] << " " << std::endl;

}

// Pickup pos
void Uarm::set_pos_pickup()
{
    float pose[] = {350, 0, _floor_load_height, 3.14, 0, 0};
    xarm_driver.arm->set_position(pose, -1, _vel, _acc, 0, false, -1, false, 0);
}

// Left pos
void Uarm::set_pos_front_left()
{
    float pose[] = {250, 250, 300, 3.14, 0, 0};
    xarm_driver.arm->set_position(pose, -1, _vel, _acc, 0, false, -1, false, 0);
}
void Uarm::set_pos_mid_left()
{
    float pose[] = {0, 250, 300, 3.14, 0, 0};
    xarm_driver.arm->set_position(pose, -1, _vel, _acc, 0, false, -1, false, 0);
}
void Uarm::set_pos_back_left()
{
    float pose[] = {-250, 250, 300, 3.14, 0, 0};
    xarm_driver.arm->set_position(pose, -1, _vel, _acc, 0, false, -1, false, 0);
}

// Right pos
void Uarm::set_pos_front_right()
{
    float pose[] = {250, -250, 300, 3.14, 0, 0};
    xarm_driver.arm->set_position(pose, -1, _vel, _acc, 0, false, -1, false, 0);
}
void Uarm::set_pos_mid_right()
{
    float pose[] = {0, -250, 300, 3.14, 0, 0};
    xarm_driver.arm->set_position(pose, -1, _vel, _acc, 0, false, -1, false, 0);
}
void Uarm::set_pos_back_right()
{
    float pose[] = {-250, -250, 300, 3.14, 0, 0};
    xarm_driver.arm->set_position(pose, -1, _vel, _acc, 0, false, -1, false, 0);
}

// Back pos
void Uarm::set_pos_back()
{
    float pose[] = {-350, 0, 300, 3.14, 0, 0};
    xarm_driver.arm->set_position(pose, -1, _vel, _acc, 0, false, -1, false, 0);
}

// Unload pos
void Uarm::set_pos_unload()
{
    float pose[] = {-350, 0, _platform_load_height, 3.14, 0, 0};
    xarm_driver.arm->set_position(pose, -1, _vel, _acc, 0, false, -1, false, 0);
}

void Uarm::rotate_pos_90()
{
    // Get current angles of all joints
    float get_angles[7];
    xarm_driver.arm->get_servo_angle(&get_angles[0]);

    std::cout << get_angles[0] << " " << get_angles[1] << " " << get_angles[2] << " " << get_angles[3] << " " << get_angles[4] << " " << get_angles[5] << " " << get_angles[6] << " " << std::endl;

    // Insert current angles to all joints except end effector
    float angles[7] = {get_angles[0], get_angles[1], get_angles[2], get_angles[3], get_angles[4], 1.57, get_angles[5]};
    xarm_driver.arm->set_servo_angle(angles, _vel_rotate, _acc_rotate, 0, true, -1, -1, false);
}

void Uarm::rotate_neg_90()
{
    // Get current angles of all joints
    float get_angles[7];
    xarm_driver.arm->get_servo_angle(&get_angles[0]);
    std::cout << get_angles[0] << " " << get_angles[1] << " " << get_angles[2] << " " << get_angles[3] << " " << get_angles[4] << " " << get_angles[5] << " " << get_angles[6] << " " << std::endl;

    // Insert current angles to all joints except end effector
    float angles[7] = {get_angles[0], get_angles[1], get_angles[2], get_angles[3], get_angles[4], -1.57, get_angles[5]};
    // xarm_driver.arm->set_servo_angle(angles, _vel_rotate, _acc_rotate, 0, true, -1, -1, false);
    xarm_driver.arm->set_servo_angle(angles, _vel, 1, 10, 0);
}

void Uarm::rotate_0()
{
    // Get current angles of all joints
    float get_angles[7];
    xarm_driver.arm->get_servo_angle(&get_angles[0]);
    std::cout << get_angles[0] << " " << get_angles[1] << " " << get_angles[2] << " " << get_angles[3] << " " << get_angles[4] << " " << get_angles[5] << " " << get_angles[6] << " " << std::endl;

    // Insert current angles to all joints except end effector
    float angles[7] = {get_angles[0], get_angles[1], get_angles[2], get_angles[3], get_angles[4], 0, get_angles[5]};
    // xarm_driver.arm->set_servo_angle(angles, _vel_rotate, _acc_rotate, 0, true, -1, -1, false);
    xarm_driver.arm->set_servo_angle(angles, _vel, 1, 10, 0);
}

void Uarm::open_gripper()
{
    xarm_driver.arm->open_lite6_gripper();
}

void Uarm::close_gripper()
{
    xarm_driver.arm->close_lite6_gripper();
}

void Uarm::stop_gripper()
{
    xarm_driver.arm->stop_lite6_gripper();
}