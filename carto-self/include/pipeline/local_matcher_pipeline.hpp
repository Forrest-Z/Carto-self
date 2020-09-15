/*
 * @Author: Liu Weilong 
 * @Date: 2020-09-04 15:16:37 
 * @Last Modified by: Liu Weilong
 * @Last Modified time: 2020-09-05 14:06:41
 */

#pragma once

#include <iostream>
#include <vector>
#include <string>


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"

namespace LwlSLAM
{
    class LidarMatcher
    {
        public:

        void pipeline();

        private:

        void initial();

        bool fetchInfo();

        void pretreatData();

        void predictPose();

        void generateMap();
        
        void match();

        bool initialFlag_;

        ros::Subscriber laserDataSub_;
        ros::Subscriber ImuInfoSub_;
        std::vector<sensor_msgs::LaserScan> laserScanPool_;
        std::vector<sensor_msgs::Imu> imuInfoPool_

    };
} // namespace name