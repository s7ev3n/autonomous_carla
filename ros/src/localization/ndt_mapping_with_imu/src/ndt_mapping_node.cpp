#include "NDTMapping.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv){
    std::cout << (CUDA_FOUND? "CUDA_FOUND":"CUDA_NOT_FOUND")<< std::endl;
    ros::init(argc, argv, "ndt_mapping_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    
}