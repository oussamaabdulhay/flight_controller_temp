#include "ros/ros.h"
#include <iostream>
#include "ROSUnit_Factory.hpp"

int main(int argc, char **argv){

    ros::init(argc, argv, "providers_node");

    ros::NodeHandle nh;
    ros::Rate rate(120);

    while(ros::ok()){
        ros::spinOnce();
        std::cout << "providers_node" << std::endl;
        rate.sleep();
    }

    return 0;
}