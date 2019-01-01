#include "LidarMapping.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv) {
    std::cout<<(CUDA_FOUND? "CUDA_FOUND":"CUDA_NOT_FOUND")<<std::endl;
    ros::init(argc, argv, "ndt_mapping");
    ros::NodeHandle nh;

    NDT::LidarMapping mapping;
    mapping.setup(nh);
    ros::Rate(10);
    ros::spin();
    return 0;
}