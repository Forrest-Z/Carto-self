/*
 * @Author: Liu Weilong 
 * @Date: 2020-08-31 21:49:04 
 * @Last Modified by: Liu Weilong
 * @Last Modified time: 2020-08-31 22:00:56
 */

#pragma once 

#include <iostream>
#include "std_msgs/Header.h"

namespace LwlSLAM
{
    class LaserData
    {
        public:
        std_msgs::Header header_;
        float            angle_min_;
        float            angle_max_;
        float            angle_increment_;
        float            time_increment_;
        
    };
} // namespace LwlSLAM