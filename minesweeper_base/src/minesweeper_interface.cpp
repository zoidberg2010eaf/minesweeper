//
// Created by joe on 4/20/19.
//

#include "minesweeper_base/minesweeper_interface.h"
#include <boost/chrono.hpp>
#include <ros/console.h>

//namespace minesweeper_base {
    double control_frequency;
    typedef boost::chrono::steady_clock time_source;
    double joints_positions_[4];  //stores readings from arduino joints callback functions
    ros::Publisher left_cmd, right_cmd;
    std_msgs::Float32 left, right;
    double test_l, test_r;
    void jointPositionsCB(const std_msgs::Float32MultiArray &joint_pos) {
        for(int i = 0; i < 4; i++){
          joints_positions_[i] = joint_pos.data[i];
        }
    }
    /*void frontLeftJointCB(const std_msgs::Float32 &pos) {
        joints_positions_[0] = pos.data;
    }

    void frontRightJointCB(const std_msgs::Float32 &pos) {
        joints_positions_[1] = pos.data;
    }

    void rearLeftJointCB(const std_msgs::Float32 &pos) {
        joints_positions_[2] = pos.data;
    }

    void rearRightJointCB(const std_msgs::Float32 &pos) {
        joints_positions_[3] = pos.data;
    }*/

    MinesweeperHardware::MinesweeperHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double control_freq) :
            nh_(nh), private_nh_(private_nh) {
        calculatePositionOffset();
        registerInterfaces();

    }

    void MinesweeperHardware::registerInterfaces() {
        for (int i = 0; i < names_.size(); i++) {
            hardware_interface::JointStateHandle joint_state_handle(names_[i], &joints_[i].pos, &joints_[i].vel,
                                                                    &joints_[i].eff);
            joint_state_interface_.registerHandle(joint_state_handle);

            hardware_interface::JointHandle joint_velocity_handle(joint_state_handle, &joints_[i].cmd);
            joint_velocity_interface_.registerHandle(joint_velocity_handle);
        }
        registerInterface(&joint_state_interface_);
        registerInterface(&joint_velocity_interface_);
    }

    void MinesweeperHardware::calculatePositionOffset() {
        for (int i = 0; i < names_.size(); i++) {
            joints_[i].pos_offset = readJointPosition(i);
        }
    }

    double MinesweeperHardware::readJointPosition(const int i) const {
        return joints_positions_[i];
    }

    double MinesweeperHardware::calculateVelocity(const int i, const ros::Duration &period) const {
        double vel = (readJointPosition(i) - joints_[i].pos) / period.toSec();
        return vel;
    }

    double MinesweeperHardware::radsToRPM(const double &rads) const {
        double rpm = (60 * rads) / (2 * PI);
        return rpm;
    }

    void MinesweeperHardware::read(const ros::Duration &period) {
        for (int i = 0; i < names_.size(); i++) {
            joints_[i].vel = calculateVelocity(i, period);
        }
        for (int i = 0; i < names_.size(); i++) {
            double delta = readJointPosition(i) - joints_[i].pos - joints_[i].pos_offset;
            joints_[i].pos += delta;
        }
    }

    void MinesweeperHardware::write(ros::NodeHandle& nh) {
        double left_cmd_vel = radsToRPM(joints_[0].cmd);
        double right_cmd_vel = radsToRPM(joints_[1].cmd);
        left_cmd = nh.advertise<std_msgs::Float32>("left_cmd", 1000);
        right_cmd = nh.advertise<std_msgs::Float32>("right_cmd", 1000);
        //code that publishes the cmd velocity
        left.data = joints_[0].cmd;
        right.data = joints_[1].cmd;
        //*l = left.data;
        //*r = right.data;
        //left.data = test_l;
        //right.data = test_r;
        left_cmd.publish(left);
        //std::cout << left << std::endl;
        //left_cmd.publish(left);
        right_cmd.publish(right);
    }

    void controlLoop(MinesweeperHardware &robot, controller_manager::ControllerManager &cm,
                     time_source::time_point &last_time, ros::NodeHandle& nh) {
        time_source::time_point current_time = time_source::now();
        boost::chrono::duration<double> elapsed_duration = current_time - last_time;
        ros::Duration elapsed(elapsed_duration.count());
        last_time = current_time;
        robot.read(elapsed);
        cm.update(ros::Time::now(), elapsed);
        robot.write(nh);
        //std::cout << robot.l << std::endl;

        //right_cmd.publish(right);
    }

    int main(int argc, char *argv[]) {
        ros::init(argc, argv, "minesweeper_base");
        ros::NodeHandle nh, private_nh("~");
        //ros::Publisher left_cmd = nh.advertise<std_msgs::Float32>("left_cmd", 1000);
        //ros::Publisher right_cmd = nh.advertise<std_msgs::Float32>("right_cmd", 1000);

        //ros::Subscriber fl_sub = nh.subscribe("front_left_wheel", 1000, frontLeftJointCB);
        ros::Subscriber joint_positions_sub = nh.subscribe("joint_positions", 1000, jointPositionsCB);
        //ros::Subscriber fr_sub = nh.subscribe("front_right_wheel", 1000, frontRightJointCB);
        //ros::Subscriber rl_sub = nh.subscribe("rear_left_wheel", 1000, rearLeftJointCB);
        //ros::Subscriber rr_sub = nh.subscribe("rear_right_wheel", 1000, rearRightJointCB);

        private_nh.param<double>("control_frequency", control_frequency, 10.0);
        MinesweeperHardware robot(nh, private_nh, control_frequency);
        controller_manager::ControllerManager cm(&robot, nh);
        ros::CallbackQueue robot_queue;
        ros::AsyncSpinner robot_spinner(1, &robot_queue);

        time_source::time_point last_time = time_source::now();
        ros::TimerOptions control_timer(
                ros::Duration(1 / control_frequency),
                boost::bind(controlLoop, boost::ref(robot), boost::ref(cm), boost::ref(last_time), boost::ref(nh)),
                &robot_queue);
        ros::Timer control_loop = nh.createTimer(control_timer);
        robot_spinner.start();

        ros::spin();
        return 0;
    }
//};
