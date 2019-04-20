//
// Created by joe on 4/20/19.
//

#ifndef MINESWEEPER_BASE_MINESWEEPER_INTERFACE_H
#define MINESWEEPER_BASE_MINESWEEPER_INTERFACE_H
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <controller_manager/controller_manager.h>
#include "ros/callback_queue.h"
#include <boost/chrono.hpp>
#include <std_msgs/Float32.h>

#define PI 3.14159265359

class MinesweeperHardware : public hardware_interface::RobotHW{
public:
    MinesweeperHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double control_freq);
    void read(const ros::Duration &period);
    void write();

private:
    ros::NodeHandle nh_, private_nh_;
    //ros::Publisher left_cmd, right_cmd;
    //std_msgs::Float32 left, right;
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface joint_velocity_interface_;
    std::vector<std::string> names_ = {"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"};
    struct Joint {
        double pos;
        double vel;
        double eff;
        double cmd;
        double pos_offset;
        Joint() : pos(0), vel(0), eff(0), cmd(0) {}
    } joints_[4];
    void calculatePositionOffset();  //set position offsets to eliminate shifts in readings
    double readJointPosition(int i) const;  //reads from arduino's published position topics
    double calculateVelocity(const int i, const ros::Duration& period) const;  //calculates velocity from position
    double radsToRPM(const double& rads) const;
    void registerInterfaces();
};

#endif //MINESWEEPER_BASE_MINESWEEPER_INTERFACE_H
