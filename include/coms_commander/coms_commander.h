#ifndef COMS_COMMANDER_H_
#define COMS_COMMANDER_H_

#include <ros/ros.h>
#include <coms_msgs/ComsGAB.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <thread>
#include <mutex>

class ComsCommander {
public:
    /* Constructors, Destructor, and Assignment operators {{{ */
    ComsCommander(const float steering_gear_ratio,
                  const float wheel_base,
                  const float update_rate);

    // Copy constructor
    ComsCommander(const ComsCommander& other) = delete;

    // Move constructor
    ComsCommander(ComsCommander&& other) = delete;

    // Destructor
    ~ComsCommander();

    // Assignment operator
    ComsCommander&
    operator=(const ComsCommander& other) = delete;

    // Move assignment operator
    ComsCommander&
    operator=(ComsCommander&& other) = delete;
    /* }}} */

    void
    set_pid(const double KP, const double KI, const double KD);

    /**
     * Starts publishing the command topics upon receiving cmd_vel
     */
    void
    begin_commanding();

    /**
     * Finish publishing the topics by joining the threads
     */
    void
    end_commanding();

    void
    odom_callback(const nav_msgs::Odometry& odom);

    void
    cmd_vel_callback(const geometry_msgs::Twist& cmd_vel);

private:
    /* Gear, Accelerator, Brake controller */
    ros::Publisher gab_pub;
    /* Steering controller */
    ros::Publisher steer_pub;
    /* Odometry */
    ros::Subscriber odom_sub;
    /* Target velocity and angular velocity */
    ros::Subscriber cmd_vel_sub;

    nav_msgs::Odometry current_odom;
    std::mutex odom_mutex;
    geometry_msgs::Twist target_twist;
    std::mutex twist_mutex;

    std::thread velocity_thread;
    std::thread steering_thread;

    float steering_gear_ratio;
    float wheel_base;
    // Really, we should be using a mutex but oh well...
    float update_rate;

    // Ideally, use a mutex
    bool is_ready;

    /* PID parameters */
    double KP;
    double KI;
    double KD;

    /* For calculating integral and derivative values */
    double err_i;
    double prev_err_p;

    /**
     * Calculates proper velocity control values and publish
     */
    void
    velocity_control();

    /**
     * Calculates proper steering control values and publish
     */
    void
    steering_control();

    /**
     * Calculates the speed from twist's x, y, z components
     * @param twist Twist message
     * @return the current speed that the vehicle is moving
     */
    double
    get_speed(const geometry_msgs::Twist& twist);
};

#endif /* end of include guard */
