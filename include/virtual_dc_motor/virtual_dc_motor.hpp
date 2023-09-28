#ifndef VIRTUAL_DC_MOTOR_HPP
#define VIRTUAL_DC_MOTOR_HPP

#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <random>

class VirtualDCMotorNode
{
public:
    VirtualDCMotorNode();

    /**
     * @brief Callback function invoked every time new steering signal is received
     *
     * @param msg  Pointer to the received message
     */
    void controlSignalCallback(const std_msgs::Int8::ConstPtr &msg);

    /**
     * @brief Main program loop resposible for virtual dc motor simulation
     *
     */
    void run();

    /**
     * @brief Function responsible for simulating non linear motor behavior
     *
     * @param cs Input Control Signal (CS)
     * @return int16_t Resulting motor position
     */
    int16_t simulateNonLinearMotorBehavior(int8_t cs);

    uint8_t generateRandomNumber();

private:
    ros::NodeHandle nh_;
    ros::Subscriber set_cs_sub_;
    ros::Publisher get_position_pub_;

    ros::Rate loop_rate_;
    int8_t cs_goal_, cs_real_;
    int16_t position_;
    int count_;

    std::mt19937 random_number_generator_ = std::mt19937(std::random_device()());
    std::uniform_int_distribution<uint8_t> random_number_distribution_ = std::uniform_int_distribution<uint8_t>(0, 100);
};

#endif