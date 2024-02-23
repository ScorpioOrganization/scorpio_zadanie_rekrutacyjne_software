#ifndef VIRTUAL_DC_MOTOR_HPP
#define VIRTUAL_DC_MOTOR_HPP

#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <random>
#include <string>

class VirtualDCMotor
{
public:
    VirtualDCMotor(ros::NodeHandle&, std::shared_ptr<std::mt19937>, const std::string);

    /**
     * @brief Callback function invoked every time new steering signal is received
     *
     * @param msg  Pointer to the received message
     */
    void controlSignalCallback(const std_msgs::Int8::ConstPtr&);

    void run();

    /**
     * @brief Function responsible for simulating non linear motor behavior
     *
     * @return int16_t Resulting motor position
     */
    void simulateNonLinearMotorBehavior();

    uint8_t generateRandomNumber();

private:
    ros::Subscriber set_cs_sub_;
    ros::Publisher get_position_pub_;

    int8_t cs_goal_, cs_real_;
    int16_t position_;
    size_t count_;

    std::shared_ptr<std::mt19937> random_number_generator_;
    std::uniform_int_distribution<uint8_t> random_number_distribution_ = std::uniform_int_distribution<uint8_t>(0, 100);
};

#endif