/*
 * @Author: Liu Weilong 
 * @Date: 2020-08-31 10:03:57 
 * @Last Modified by: Liu Weilong
 * @Last Modified time: 2020-09-04 16:11:46
 */


#pragma once

#include <iostream>
#include <string>
#include <deque>
#include <memory>
#include <vector>

#include "config.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "yaml-cpp/yaml.h"
#include "voxel_filter.hpp"

#include "rosbag/bag.h"
#include "rosbag/view.h"

namespace LwlSLAM
{
    class RawDataProcessor
    {
        public:

        RawDataProcessor(){}
        
        /**
         * @brief 函数主体调用deskew、downsampling、pubInfo
         */
        void pipeline();

        private:

        /**
         * @brief 初始化参数、publisher、subscriber
         */
        void initial();

        /**
         * @brief 根据时间从两个数据库中提取最新的LaserScan
         */
        bool getCurrentLaserScan();

        void deskew();

        void downSample();

        void pubInfo();

        void clearMidProduct();

        /**
         * @brief 使用Protobuffer 进行序列化 (还在思考要不要开一个线程来进行序列化)
         *        目前是在考虑算法的实时性的问题
         *        还是决定先单线程使用，主要是为了练手Protobuffer
         */
        void midProductSave();
        
        void laserDataSubCallback(const sensor_msgs::LaserScanConstPtr msg){ laserDataPool_.push_back(*msg);
        ROS_DEBUG("the received Laser seq is %d ", laserDataPool_.front().header.seq);
        }
        
        void imuInfoSubCallback(const sensor_msgs::ImuConstPtr msg){ imuInfoPool_.push_back(*msg);}

#ifdef ROS_BAG_INPUT      
        void rosBagIO(sensor_msgs::LaserScan & output);
#endif 

        float voxelFilterSize_1;
        float voxelFilterSize_2;

        VoxelFilter downSampler_;

        std::deque<sensor_msgs::LaserScan> laserDataPool_;
        std::deque<sensor_msgs::Imu> imuInfoPool_;

        ros::NodeHandle nh_;
        ros::Publisher imuInfoPub_;
        ros::Publisher rawLaserDataPub_;
        ros::Publisher sparseLaserDataPub_;
        ros::Subscriber rawLaserDataSub_;
        ros::Subscriber rawImuInfoSub_;

        sensor_msgs::LaserScan currentLaserScan_;
        sensor_msgs::LaserScan sparseLaserScan_;
        std::vector<sensor_msgs::ImuConstPtr> currentImuInfo_;

        
        bool usingIMU_;
        bool initialFlag_;
        bool midProductSave_;
        
        bool initialBag_;
    };
}



