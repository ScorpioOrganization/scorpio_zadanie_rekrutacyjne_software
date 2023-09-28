#include "virtual_dc_motor.hpp"

VirtualDCMotorNode::VirtualDCMotorNode() : loop_rate_(400),
                                           cs_goal_(0),
                                           cs_real_(0),
                                           position_(0),
                                           count_(0)
{
    nh_ = ros::NodeHandle("virtual_dc_motor");
    set_cs_sub_ = nh_.subscribe("/virtual_dc_motor/set_cs", 10, &VirtualDCMotorNode::controlSignalCallback, this);
    get_position_pub_ = nh_.advertise<std_msgs::UInt16>("/virtual_dc_motor/get_position", 10);
}

void VirtualDCMotorNode::controlSignalCallback(const std_msgs::Int8::ConstPtr &msg)
{
    if (msg->data >= -100 && msg->data <= 100)
    {
        cs_goal_ = msg->data;
    }
}

void VirtualDCMotorNode::run()
{
    while (ros::ok())
    {
        position_ += simulateNonLinearMotorBehavior(cs_goal_);
        if (count_ % 4 == 0)
        {
            std_msgs::UInt16 position_msg;
            uint8_t random_number = generateRandomNumber();
            if (random_number < 30)
            {
                position_msg.data = position_ - 1;
            }
            else if (random_number > 70)
            {
                position_msg.data = position_ + 1;
            }
            else
            {
                position_msg.data = position_;
            }
            if (position_msg.data >= 0)
            {
                position_msg.data = position_msg.data % 4096;
            }
            else
            {
                position_msg.data += 4095;
            }
            get_position_pub_.publish(position_msg);
        }
        count_++;
        ros::spinOnce();
        loop_rate_.sleep();
    }
}

int16_t VirtualDCMotorNode::simulateNonLinearMotorBehavior(int8_t cs)
{
    double k = 0.006;
    if (cs >= 0)
    {
        return static_cast<int16_t>(k * cs * cs);
    }
    else
    {
        return -static_cast<int16_t>(k * cs * cs);
    }
}

uint8_t VirtualDCMotorNode::generateRandomNumber()
{
    return random_number_distribution_(random_number_generator_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "virtual_dc_motor_node");
    VirtualDCMotorNode node;
    node.run();
    return 0;
}