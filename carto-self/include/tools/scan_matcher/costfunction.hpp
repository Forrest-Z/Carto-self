/*
 * @Author: Liu Weilong
 * @Date: 2020-09-16 06:57:39
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-09-16 07:29:32
 * @Description: 包含CostFunction的头文件
 */

#pragma once
#include <vector>
#include "ceres/ceres.h"
#include "Eigen/Eigen"
#include "glog/logging.h"
#include "mapping_structure.hpp"
namespace LwlSLAM
{
    class OccupiedSpaceMatchError
    {
        OccupiedSpaceMatchError(float scaleFactor,
        const std::vector<Eigen::Vector2f> & laserInfo,
        const ProbabilityGridMap & submap):scaleFactor_(scaleFactor),
        laserInfo_(laserInfo),submap_(submap)
        {
            CHECK_NE(laserInfo_.size(),0);
            CHECK_NE(submap_.getCorrespendenceValueTable().size(),0);
        }
        template <typename T>
        bool operator()(const T * const pose_,T* residual)const {
            
        } 

        private:
        float scaleFactor_;
        const std::vector<Eigen::Vector2f> & laserInfo_;
        const ProbabilityGridMap & submap_;
    };
} // namespace LwlSLAM

