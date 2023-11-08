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

    // Gripper commands
    void open_gripper();
    void close_gripper();
    void stop_gripper();

private:
    xarm_api::XArmDriver xarm_driver;
    int _vel = 400;
    int _acc = 50;
    int _platform_load_height = 160;
    int _floor_load_height = -50;
};

