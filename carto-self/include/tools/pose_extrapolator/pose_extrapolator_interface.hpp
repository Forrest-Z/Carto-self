/*
 * @Author: Liu Weilong
 * @Date: 2020-09-11 12:57:54
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-09-11 14:32:40
 * @Description: 位姿推断基类
 */

#pragma once 

#include <iostream>
#include <vector>

#include "Eigen/Eigen"

#include "matching_structure.hpp"

namespace LwlSLAM
{
    class PoseExtrapolatorInterface
    {
        public:
        
        virtual void pipeline()=0;
        
        virtual void initial()=0;

        virtual void insertNewPose(PoseInfo) =0;

        virtual PoseInfo predictPose() =0;
        
        std::vector<PoseInfo> poseArray_;

    };
}