/*
 * @Author: Liu Weilong 
 * @Date: 2020-09-01 16:16:30 
 * @Last Modified by: Liu Weilong
 * @Last Modified time: 2020-09-04 19:01:16
 */

#pragma once
#include <vector>
#include <bitset>
#include <unordered_set>
#include <string>


#include "down_sampler_interface.hpp"
#include "sensor_msgs/LaserScan.h"
#include "eigen3/Eigen/Eigen"
#include "sophus/geometry.hpp"

namespace LwlSLAM
{
    class VoxelFilter:public DownSamplingInterface<sensor_msgs::LaserScan>
    {
        public:
        using SensorType = sensor_msgs::LaserScan;
        using MidProduct = std::vector<Eigen::Vector3f>;

        VoxelFilter(const float gridMapsize = 0.05):gridSize_(gridMapsize){ std::cout<<"ctor used the gridSize is "<<gridSize_<<std::endl;}
        

        virtual void pipeline(const SensorType & Input,SensorType & Output) override ;

        void setGridSize(const float gridSize){gridSize_ = gridSize;} 

        void clearMidProduct(){tmpMidProductSparcePoint_.clear(); tmpMidProductRawPoint_.clear();}

        const float getGridSize(){return gridSize_;}

        const MidProduct & getMidProductRawPoint()const{return tmpMidProductRawPoint_;} 

        const MidProduct & getMidProductSparsePoint() const {return tmpMidProductSparcePoint_;}
        
        private:

        void dataPrepare(const SensorType & Input);
        
        void downSample(const SensorType & Input,SensorType & Output) ;

        float gridSize_;
        
        MidProduct tmpMidProductRawPoint_;
        MidProduct tmpMidProductSparcePoint_;
    };
} // namespace LwlSLAM
