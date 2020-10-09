/*
 * @Author: Liu Weilong 
 * @Date: 2020-09-05 14:02:43 
 * @Last Modified by: Liu Weilong
 * @Last Modified time: 2020-09-05 14:09:38
 */


#pragma once    

#include <iostream>
#include <vector>
#include <string>

#include "Eigen/Eigen"
#include "sensor_msgs/LaserScan.h"
#include "yaml-cpp/yaml.h"

#include "common/common.hpp"
#include "tools/structure/mapping_structure.hpp"


namespace LwlSLAM
{
    class GridMapGenerator2d
    {
        public:

        enum DrawStatus
        {
            NoDrawingBefore = 0,
            CommonDraw,
            OneSubmapComplete
        };
        
        GridMapGenerator2d()=delete;

        explicit GridMapGenerator2d(const std::string & configFile):configFile_(configFile_){}

        void pipeline(sensor_msgs::LaserScan& laserInfo,Eigen::Vector3f pose);


        const float getPixelSize() const
        {
            return pixelSize_;
        }
        const ProbabilityGridMap & getGridMap(uint16_t seq){
            CHECK(seq<submapPool_.size());
            return submapPool_[seq];
        }

        const ProbabilityGridMap & getCurrentMap(){
        if (submapPool_.size()>1)
        return submapPool_[submapPool_.size()-2];
        else
        return submapPool_.back();
        }

        const unsigned int getPoolSize()const
        {
            return submapPool_.size();
        }


        private:

        void initial();

        void dataPrepare(sensor_msgs::LaserScan& laserInfo , Eigen::Vector3f pose);

        void drawLaserPipeLine();
        
        void saveSubmap();

        //------------------------------------------------------------------------------------------------

        DrawStatus checkStatus();

        /**
         * @brief 用于
        */
        void finish(ProbabilityGridMap & submap);

        void findMinRect(Eigen::AlignedBox2i & scale);

        void castRays(ProbabilityGridMap & submap);

        /**
         * @brief 这里的origin是已经处理过的在submap 中的坐标，end 也是submap中的坐标
        */
        void castRay(const Eigen::Array2i origin, const Eigen::Array2i end, ProbabilityGridMap & submap);

        bool initialFlag_;

        float pixelSize_;

        float discreteInterval_;

        int submapCapacity_;

        int checkCapacity_;

        std::string configFile_;

        Eigen::Array2i tmpMidProductPose_;
        
        std::vector<Eigen::Array2i> tmpMidProductDiscretion_;

        std::vector<Eigen::Vector3f> tmpMidProductXYZ_;

        std::vector<ProbabilityGridMap> submapPool_;

        
    };  
} // namespace LwlSLAM
