#include "virtual_dc_motor.hpp"

VirtualDCMotor::VirtualDCMotor(ros::NodeHandle& nh, std::shared_ptr<std::mt19937> random_number_generator, const std::string suffix) :
                                           cs_goal_(0),
                                           cs_real_(0),
                                           position_(0),
                                           count_(0),
                                           random_number_generator_(random_number_generator)
{
    set_cs_sub_ = nh.subscribe("/virtual_dc_motor/set_cs_" + suffix, 10, &VirtualDCMotor::controlSignalCallback, this);
    get_position_pub_ = nh.advertise<std_msgs::UInt16>("/virtual_dc_motor/get_position_" + suffix, 10);
}

void VirtualDCMotor::controlSignalCallback(const std_msgs::Int8::ConstPtr &msg)
{
    if (msg->data >= -100 && msg->data <= 100)
    {
        cs_goal_ = msg->data;
    }
}

void VirtualDCMotor::run()
{
    simulateNonLinearMotorBehavior();
    if (count_++ % 4 == 0)
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
        position_msg.data %= 4096;
        get_position_pub_.publish(position_msg);
    }
}

void VirtualDCMotor::simulateNonLinearMotorBehavior()
{
    position_ += static_cast<int16_t>(0.006 * cs_goal_ * cs_goal_) * (cs_goal_ >= 0 ? 1 : -1);
}

uint8_t VirtualDCMotor::generateRandomNumber()
{
    return random_number_distribution_(*random_number_generator_);
}

bool motoConnectionServiceCallback() {

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "virtual_dc_motor_node");
    std::shared_ptr<std::mt19937> random_number_generator = std::make_shared<std::mt19937>(std::random_device()());
    ros::NodeHandle nh("virtual_dc_motor_node");

    // ros::ServiceServer motor_connection = nh.advertiseService("", motoConnectionServiceCallback);

    VirtualDCMotor motor0(nh, random_number_generator, "0");
    VirtualDCMotor motor1(nh, random_number_generator, "1");
    VirtualDCMotor motor2(nh, random_number_generator, "2");

    ros::Rate rate(40);
    while(ros::ok()) {
        motor0.run();
        motor1.run();
        motor2.run();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
