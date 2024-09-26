#include "autonomy_simulator.hpp"

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <vector>
#include "autonomy_simulator/RoverPose.h"
#include "autonomy_simulator/RoverMap.h"
#include "autonomy_simulator/GetMap.h"
#include "map_generation.hpp"
#include "utils.hpp"

#define GRID_SIZE 50

void AutonomySimulator::roverPoseCallback(const ros::TimerEvent&) {
    autonomy_simulator::RoverPose msg;
    if(_illegalState) {
        msg.x = -1;
        msg.y = -1;
        msg.orientation = -1;
    }
    else {
        msg.x = _roverPoseX;
        msg.y = _roverPoseY;
        msg.orientation = _roverPoseR;
    }
    _roverPosePublisher.publish(msg);
}

bool AutonomySimulator::getMapServiceCallback(
    autonomy_simulator::GetMap::Request&,
    autonomy_simulator::GetMap::Response& response) {
    if(_illegalState) {
        return false;
    }
    response.data.clear();
    std::for_each(_map.begin(), _map.end(), [&](const std::vector<int8_t> &row) {
        response.data.insert(response.data.end(), row.begin(), row.end());
    });
    return true;
}

void AutonomySimulator::roverMapCallback(const ros::TimerEvent&) {
    const std::vector<std::pair<int8_t, int8_t>> sensorFOV = {
        { -1, 1 }, { 0, 1 }, { 1, 1 },
        { -1, 2 }, { 0, 2 }, { 1, 2 },
    };
    autonomy_simulator::RoverMap msg;
    msg.data.clear();
    for(const auto &delta : sensorFOV) {
        const auto deltaInDir = deltaInDirection(delta, _roverPoseR);
        int8_t val = 100;
        const std::pair<int8_t, int8_t> fieldPos = { _roverPoseX + deltaInDir.first, _roverPoseY + deltaInDir.second };
        if(fieldPos.first >= 0 && fieldPos.first < GRID_SIZE
        && fieldPos.second >= 0 && fieldPos.second < GRID_SIZE) {
            val = _map[fieldPos.second][fieldPos.first];
        }
        msg.data.push_back(val);
    }
    _roverMapPublisher.publish(msg);
}

void AutonomySimulator::roverMoveCallback(const std_msgs::UInt8& move) {
    if(_illegalState) {
        return;
    }
    int8_t moveDirection = -1;
    switch (move.data) {
        case 0:
            _roverPoseR = (_roverPoseR + 3) % 4;
            break;
        case 1:
            _roverPoseR = (_roverPoseR + 1) % 4;
            break;
        case 2:
            moveDirection = 1;
        case 3:
            const auto positionDelta = deltaInDirection({ 0, moveDirection }, _roverPoseR);
            const int8_t previouseHeight = _map[_roverPoseY][_roverPoseX];
            _roverPoseX += positionDelta.first;
            _roverPoseY += positionDelta.second;
            if(_roverPoseX < 0 || _roverPoseX >= GRID_SIZE
            || _roverPoseY < 0 || _roverPoseY >= GRID_SIZE
            || abs(_map[_roverPoseY][_roverPoseX] - previouseHeight) > 10) {
                _illegalState = true;
            }
            break;
    }
}

AutonomySimulator::AutonomySimulator():
    _nh(ros::NodeHandle("autonomy_simulator")),
    _generateObstacles(_nh.param(std::string("generate_obstacles"), false)),
    _roverPoseX(0),
    _roverPoseY(0),
    _roverPoseR(autonomy_simulator::RoverPose::ORIENTATION_NORTH),
    _illegalState(false),
    _roverPosePublisher(_nh.advertise<autonomy_simulator::RoverPose>("/rover/pose", 1)),
    _roverPosePublisherTimer(_nh.createTimer(
        ros::Duration(0.1),
        std::bind(&AutonomySimulator::roverPoseCallback, this, std::placeholders::_1),
        false,
        false)),
    _roverMoveSubscriber(_nh.subscribe("/rover/move", 1, &AutonomySimulator::roverMoveCallback, this)) {
        if(_generateObstacles) {
            _map = map_generation::genrateRandomMap(GRID_SIZE).getMap();
            _getMapService = _nh.advertiseService("/get_map", &AutonomySimulator::getMapServiceCallback, this);
            _roverMapPublisher = _nh.advertise<autonomy_simulator::RoverMap>("/rover/sensor", 1);
            _roverMapPublisherTimer = _nh.createTimer(
                ros::Duration(0.1),
                std::bind(&AutonomySimulator::roverMapCallback, this, std::placeholders::_1),
                false,
                false);
        }
        else {
            _map = std::vector<std::vector<int8_t>>(GRID_SIZE, std::vector<int8_t>(GRID_SIZE, 0));
        }
    }

void AutonomySimulator::start() {
    _roverPosePublisherTimer.start();
    if(_generateObstacles) {
        _roverMapPublisherTimer.start();
    }
    ros::spin();
}
