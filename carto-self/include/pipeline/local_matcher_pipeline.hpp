/*
 * @Author: Liu Weilong 
 * @Date: 2020-09-04 15:16:37 
 * @Last Modified by: Liu Weilong
 * @Last Modified time: 2020-09-05 14:06:41
 */

#pragma once

#include <iostream>
#include <vector>
#include <deque>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"

#include "tools/submap_generator/2d_gridmap_generator.hpp"
#include "tools/scan_matcher/2d_scan_matcher.hpp"
#include "tools/pose_extrapolator/linear_estimator.hpp"


namespace LwlSLAM
{
    class LidarMatcher
    {
        public:

        LidarMatcher(const std::string configFile,const std::string submapGeneratorConfig,
        const std::string matcherConfig):configFile_(configFile),
        submapGenerator_(submapGeneratorConfig),matcher_(matcherConfig),nh_("~"){
            
        }

        void pipeline();

        private:

        void initial();

        bool fetchInfo();

        void pretreatData();

        void predictPose();

        void generateMap();
        
        void match();

        void laserinfoCb(const sensor_msgs::LaserScan::ConstPtr  msg);

        void imuinfoCb(const sensor_msgs::ImuConstPtr  msg);

        bool initialFlag_;

        const std::string configFile_;
        
        GridMapGenerator2d submapGenerator_;
        CorrelativeScanMatcher2d matcher_;
        LinearEstimator poseExtrapolator_;
        ros::NodeHandle nh_;
        ros::Subscriber laserInfoSub_;
        ros::Subscriber ImuInfoSub_;
        ros::Publisher  posePub_;
        std::deque<sensor_msgs::LaserScan> laserScanPool_;
        std::deque<sensor_msgs::Imu> imuInfoPool_;
        sensor_msgs::LaserScan currentScan_;
        PoseInfo predictPose_;
        // 这里对应的就是matcher 之后产出的地图
        PoseInfo predictPoseUpdate_;

    };
} // namespace name