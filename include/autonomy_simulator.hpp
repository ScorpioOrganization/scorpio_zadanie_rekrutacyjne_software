#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <vector>
#include "autonomy_simulator/RoverPose.h"
#include "autonomy_simulator/RoverMap.h"
#include "autonomy_simulator/GetMap.h"
#include "utils.hpp"

#define GRID_SIZE 50

class AutonomySimulator {
    ros::NodeHandle _nh;

    // Simulation
    int8_t _roverPoseX;
    int8_t _roverPoseY;
    autonomy_simulator::RoverPose::_orientation_type _roverPoseR;
    std::vector<std::vector<int8_t>> _map;
    bool _illegalState;

    // RoverPose Publisher
    ros::Publisher _roverPosePublisher;
    ros::Timer _roverPosePublisherTimer;
    void roverPoseCallback(const ros::TimerEvent&);

    // GetMap Service
    ros::ServiceServer _getMapService;
    bool getMapServiceCallback(autonomy_simulator::GetMap::Request&, autonomy_simulator::GetMap::Response&);

    // RoverMap Publisher
    ros::Publisher _roverMapPublisher;
    ros::Timer _roverMapPublisherTimer;
    void roverMapCallback(const ros::TimerEvent&);

    // RoverMove subscriber
    ros::Subscriber _roverMoveSubscriber;
    void roverMoveCallback(const std_msgs::UInt8&);

 public:
    AutonomySimulator();
    void start();
};
