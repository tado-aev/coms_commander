#include "coms_commander/coms_commander.h"

#include <ros/ros.h>

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "coms_commander_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p{"~"};

    float steering_gear_ratio;
    float wheel_base;
    float update_rate;
    double KP, KI, KD;

    nh_p.getParam("steering_gear_ratio", steering_gear_ratio);
    nh_p.getParam("wheel_base", wheel_base);
    nh_p.getParam("update_rate", update_rate);
    nh_p.getParam("KP", KP);
    nh_p.getParam("KI", KI);
    nh_p.getParam("KD", KD);

    ComsCommander commander{steering_gear_ratio,
                            wheel_base,
                            update_rate};
    commander.set_pid(KP, KI, KD);
    commander.begin_commanding();

    ros::spin();

    commander.end_commanding();

    return 0;
}
