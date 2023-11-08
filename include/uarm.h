#include "xarm_api/xarm_driver.h"

class Uarm
{
public:
    Uarm(rclcpp::Node::SharedPtr& node, std::string &robot_ip);
    ~Uarm();

    // Home pos
    void set_pos_home();

    // Lookout pos
    void set_pos_lookout();

    // Pickup pos
    void set_pos_pickup();

    // Left pos
    void set_pos_front_left();
    void set_pos_mid_left();
    void set_pos_back_left();  

    // Right pos
    void set_pos_front_right();
    void set_pos_mid_right();
    void set_pos_back_right();

    // Back pos
    void set_pos_back();

    // Unload pos
    void set_pos_unload();

    // Rotate commands
    void rotate_pos_90();
    void rotate_neg_90();
    void rotate_0();

    // Gripper commands
    void open_gripper();
    void close_gripper();
    void stop_gripper();

private:
    xarm_api::XArmDriver xarm_driver;
    float _vel = 1000;
    float _acc = 150;

    float _vel_rotate = 10;
    float _acc_rotate = 1;

    float _platform_load_height = 165;
    float _floor_load_height = -50;
};

