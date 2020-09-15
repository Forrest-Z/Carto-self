/*
 * @Author: Liu Weilong 
 * @Date: 2020-08-26 06:13:45 
 * @Last Modified by: Liu Weilong
 * @Last Modified time: 2020-09-04 08:29:09
 */

#include <iostream>

#include "ros/ros.h"
#include "data_processor_pipeline.hpp"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"raw_data_processor_node");
    
    LwlSLAM::RawDataProcessor dataProcessor_;

    dataProcessor_.pipeline();


    return 0;
}