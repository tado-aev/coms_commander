#include "coms_commander/coms_commander.h"

/* Constructors, Destructor, and Assignment operators {{{ */
ComsCommander::ComsCommander(const float steering_gear_ratio,
                             const float wheel_base,
                             const float update_rate)
    : steering_gear_ratio{steering_gear_ratio}
    , wheel_base{wheel_base}
    , update_rate{update_rate}
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
}

void
ComsCommander::velocity_control() {
    auto r = ros::Rate{update_rate};
    while (ros::ok()) {
        r.sleep();

        if (!is_ready) {
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

        coms_msgs::ComsGAB msg;
        msg.gear = "d";
        msg.accel = cmd_percentage;
        // TODO: programmatically determine brake
        msg.brake = 0;
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

        // TODO: steering control, but would it be on receiving twist?
        twist_mutex.lock();
        auto tgt_yaw = target_twist.angular.z;
        twist_mutex.unlock();

        odom_mutex.lock();
        auto v_cur = get_speed(current_odom.twist.twist);
        odom_mutex.unlock();

        auto tire_deg = std::atan2(tgt_yaw * wheel_base, v_cur);
        auto steering_deg = tire_deg * steering_gear_ratio;

        std_msgs::Float64MultiArray msg;
        // Position (rad)
        msg.data.push_back(steering_deg);
        // Velocity (rad/s)
        // TODO: obviously, make these adjustable
        msg.data.push_back(3.14159265358979324);
        // Acceleration (rad/s^2)
        msg.data.push_back(0.69813170077777775);
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