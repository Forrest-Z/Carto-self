/*
 * @Author: Liu Weilong
 * @Date: 2020-09-15 07:28:51
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-09-16 07:07:18
 * @Description: 优化匹配模块
 */

#pragma once
#include <iostream>
#include <vector>
#include "ceres/ceres.h"
#include "mapping_structure.hpp"

namespace LwlSLAM
{
    class CeresMatcher
    {
        public:

        CeresMatcher(float translationWeight, float rotationWeight,
        const ProbabilityGridMap & submap);

        void Solve();

        const std::vector<float> & getResult()const;
        private:

        float translationWeight_;
        float rotationWeight_;
        std::vector<float> result_;        
        const ProbabilityGridMap & submap_;
        
    };
} // namespace name
