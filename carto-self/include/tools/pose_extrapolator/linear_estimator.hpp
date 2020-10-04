/*
 * @Author: Liu Weilong
 * @Date: 2020-09-11 13:15:23
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-10-02 13:15:56
 * @Description: 一个线性位姿估计器
 */

#pragma once
#include <math.h>

#include "glog/logging.h"

#include "tools/pose_extrapolator/pose_extrapolator_interface.hpp"
#include "tools/structure/matching_structure.hpp"

namespace LwlSLAM
{
    class LinearEstimator:public PoseExtrapolatorInterface
    {
        public:

        LinearEstimator(){}

        void pipeline(){}

        void initial();

        unsigned int getArraySize(){return poseArray_.size();}

        PoseInfo predictPose(uint32_t sec, uint32_t nsec) override;

        void insertNewPose(PoseInfo) override;
        
        private:

        using PoseExtrapolatorInterface::poseArray_;
    
    };
}