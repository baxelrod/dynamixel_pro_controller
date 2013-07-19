#ifndef DYNAMIXEL_PRO_CONTROLLER_H_
#define DYNAMIXEL_PRO_CONTROLLER_H_

#include <string>
#include <map>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <dynamixel_pro_driver/dynamixel_pro_driver.h>

namespace dynamixel_pro_controller
{

class DynamixelProController
{
public:
    DynamixelProController();
    ~DynamixelProController();

    void startListeningForCommands();
    void startBroadcastingJointStates();
private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

    void publishJointStates(const ros::TimerEvent& e);

    ros::NodeHandle *nh;
    dynamixel_pro_driver::DynamixelProDriver *driver;
    double publish_rate;
    bool publish_velocities;

    bool shutting_down;

    struct dynamixel_spec
    {
        std::string name;
        uint16_t model_number;
        int cpr;
        double gear_reduction;
    };

    struct dynamixel_info 
    {
        int id;
        std::string joint_name;
        std::string model_name;
        uint16_t model_number;
        uint32_t model_info;
        int cpr;
        double gear_reduction;
    };

    enum control_mode
    {
        POSITION_CONTROL = 3,
        VELOCITY_CONTROL = 1,
        TORQUE_CONTROL = 0,
        UNKOWN  = -1
    };

    struct dynamixel_status
    {
        int id;
        control_mode mode; 
        bool torque_enabled;
    };

    std::map<std::string, dynamixel_info> joint2dynamixel;
    std::map<uint16_t, dynamixel_spec> model_number2specs;
    std::map<int, dynamixel_status> id2status;

    ros::Publisher jointStatePublisher;
    ros::Subscriber jointStateSubscriber;
    ros::Timer broadcastTimer;
};

}

#endif //DYNAMIXEL_PRO_CONTROLLER_H_
