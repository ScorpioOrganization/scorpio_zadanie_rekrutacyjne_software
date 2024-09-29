#include <ros/ros.h>
#include "autonomy_simulator.hpp"

int main(int argc, char** const argv) {
    ros::init(argc, argv, "autonomy_simulator");
    AutonomySimulator autonomySimulator;
    autonomySimulator.start();
    return 0;
}
