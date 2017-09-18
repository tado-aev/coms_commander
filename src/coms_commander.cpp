#include "coms_commander/coms_commander.h"

/* Constructors, Destructor, and Assignment operators {{{ */
ComsCommander::ComsCommander(const float steering_gear_ratio,
                             const float wheel_base,
                             const float update_rate,
                             const std::vector<float>& steering_vel,
                             const float cmd_ang_acc)
    : first_twist_received{false}
    , steering_gear_ratio{steering_gear_ratio}
    , wheel_base{wheel_base}
    , update_rate{update_rate}
    , steering_vel{steering_vel}
    , cmd_ang_acc{cmd_ang_acc}
    , is_ready{false}
{
    ros::NodeHandle nh;
    gab_pub = nh.advertise<coms_msgs::ComsGAB>("cmd_gab", 100);
    steer_pub = nh.advertise<std_msgs::Float64MultiArray>("cmd_steer", 100);
    odom_sub = nh.subscribe("odom",
                            100,
                            &ComsCommander::odom_callback,
                            this);
    cmd_vel_sub = nh.subscribe("cmd_vel",
                               100,
                               &ComsCommander::cmd_vel_callback,
                               this);
}

// Destructor
ComsCommander::~ComsCommander()
{
    if (velocity_thread.joinable()) {
        velocity_thread.join();
    }
    if (steering_thread.joinable()) {
        steering_thread.join();
    }
}
/* }}} */

void
ComsCommander::set_pid(const double KP, const double KI, const double KD) {
    this->KP = KP;
    this->KI = KI;
    this->KD = KD;
    err_i = 0;
    prev_err_p = 0;
}

void
ComsCommander::begin_commanding() {
    is_ready = true;

    velocity_thread = std::thread{&ComsCommander::velocity_control, this};
    steering_thread = std::thread{&ComsCommander::steering_control, this};
}

void
ComsCommander::end_commanding() {
    is_ready = false;

    if (velocity_thread.joinable()) {
        velocity_thread.join();
    }
    if (steering_thread.joinable()) {
        steering_thread.join();
    }
}

void
ComsCommander::odom_callback(const nav_msgs::Odometry& odom) {
    std::lock_guard<std::mutex> lock{odom_mutex};
    current_odom = odom;
}

void
ComsCommander::cmd_vel_callback(const geometry_msgs::Twist& cmd_vel) {
    std::lock_guard<std::mutex> lock{twist_mutex};
    target_twist = cmd_vel;
    first_twist_received = true;
}

void
ComsCommander::velocity_control() {
    auto r = ros::Rate{update_rate};
    while (ros::ok()) {
        r.sleep();

        if (!is_ready || !first_twist_received) {
            continue;
        }

        odom_mutex.lock();
        auto v_cur = get_speed(current_odom.twist.twist);
        odom_mutex.unlock();

        twist_mutex.lock();
        auto v_tgt = get_speed(target_twist);
        twist_mutex.unlock();

        auto err_p = v_tgt - v_cur;
        this->err_i += err_p;
        auto err_d = err_p - prev_err_p;
        double cmd_percentage = KP * err_p + KI * err_i + KD * err_d;
        // Limit to [-100, 100]
        cmd_percentage = cmd_percentage > 100 ? 100 : cmd_percentage;
        cmd_percentage = cmd_percentage < -100 ? -100 : cmd_percentage;

        coms_msgs::ComsGAB msg;
        msg.gear = "d";
        if (cmd_percentage > 0) {
            msg.accel = cmd_percentage;
            msg.brake = 0;
        }
        else {
            msg.accel = 0;
            msg.brake = -cmd_percentage;
        }
        gab_pub.publish(msg);
    }
}

void
ComsCommander::steering_control() {
    auto r = ros::Rate{update_rate};
    while (ros::ok()) {
        r.sleep();

        if (!is_ready) {
            continue;
        }

        twist_mutex.lock();
        auto tgt_omega = target_twist.angular.z;
        twist_mutex.unlock();

        odom_mutex.lock();
        auto cur_vel = get_speed(current_odom.twist.twist);
        odom_mutex.unlock();

        if (cur_vel == 0) {
            continue;
        }

        auto tire_ang = std::atan2(tgt_omega * wheel_base, cur_vel);
        auto cmd_ang = tire_ang * steering_gear_ratio;
        auto cmd_ang_vel = get_steering_speed(cur_vel, cmd_ang);
        ROS_INFO_STREAM(180 / M_PI * tgt_omega);
        ROS_INFO_STREAM("  " << 180 / M_PI * cmd_ang);
        ROS_INFO_STREAM("  " << 180 / M_PI * cmd_ang_vel);
        ROS_INFO_STREAM("  " << 180 / M_PI * cmd_ang_acc);

        if (cmd_ang_vel == 0) {
            continue;
        }

        std_msgs::Float64MultiArray msg;
        // Position (rad)
        msg.data.push_back(cmd_ang);
        // Velocity (rad/s)
        msg.data.push_back(cmd_ang_vel);
        // Acceleration (rad/s^2)
        msg.data.push_back(cmd_ang_acc);
        steer_pub.publish(msg);
    }
}

double
ComsCommander::get_speed(const geometry_msgs::Twist& twist) {
    auto v_x = twist.linear.x;
    auto v_y = twist.linear.y;
    auto v_z = twist.linear.z;
    auto v_xy = std::sqrt(std::pow(v_x, 2) + std::pow(v_y, 2));
    auto v = std::sqrt(std::pow(v_xy, 2) + std::pow(v_z, 2));
    return static_cast<double>(v);
}

double
ComsCommander::get_steering_speed(const double v_cur, const double cmd_ang) {
    if (v_cur < steering_vel[0]) {
        return 0;
    }
    if (v_cur > steering_vel[1]) {
        return steering_vel[2];
    }

    auto slope = steering_vel[2] / (steering_vel[1] - steering_vel[0]);
    return v_cur * slope;
}
