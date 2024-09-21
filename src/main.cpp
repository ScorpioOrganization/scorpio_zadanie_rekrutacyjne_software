#include <ros/ros.h>
#include "autonomy_simulator.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "autonomy_simulator");
    AutonomySimulator as;
    as.start();
    return 0;
}
