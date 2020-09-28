/*
 * @Author: Liu Weilong
 * @Date: 2020-09-11 12:57:54
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-09-27 07:12:39
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

        PoseExtrapolatorInterface() =default;

        virtual ~PoseExtrapolatorInterface(){}
        
        virtual void pipeline()=0;
        
        virtual void initial()=0;

        virtual void insertNewPose(PoseInfo) =0;

        virtual PoseInfo predictPose(uint32_t sec, uint32_t nsec) =0;
        
        std::vector<PoseInfo> poseArray_;

    };
}