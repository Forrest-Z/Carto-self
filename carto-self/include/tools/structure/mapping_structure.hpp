/*
 * @Author: Liu Weilong
 * @Date: 2020-09-05 15:39:10
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-10-04 22:38:24
 * @Description: 文件用于定义 地图的类型 目前之后 概率栅格地图
 */


#pragma once
#include <iostream>
#include <vector>
#include <string>

#include "Eigen/Eigen"
#include "glog/logging.h"
#include "yaml-cpp/yaml.h"

namespace LwlSLAM
{


    /**
     * @brief 有关于地图概率的参数 
     */
    struct ProbabilityParameters
    {
        float lowBound;
        float highBound;
        float hitOdd;
        float missOdd;
        float unknown;
    };

    /**
     * @brief 不能进行单独使用，请结合ProbabilityGridMap 进行使用
     */
    // TODO gird_size_ 需要被添加进MapLimits
    class MapLimits
    {
        public:

        const Eigen::Vector2i& getLimits() const{return scaleXY_;}

        void setLimits(int x, int y);

        int getCellIndex(int x, int y) const;

        bool contain(int x, int y) const;
        
        template<typename T>
        bool contain(T xy) const{
            int x = xy(0);
            int y = xy(1);
            return (x<scaleXY_.x())&&(y<scaleXY_.y())&&(x>=0)&&(y>=0);
        }
        private:

        double gird_size_;

        Eigen::Vector2i scaleXY_;
    };

    /**
     * @brief 用于数据的存储
     */

    class ProbabilityGridMap
    {
      public:

        /**
         * @brief 对外接口
         */
        
        ProbabilityGridMap(int seq):seq_(seq){

            checkInitialization();
            CorrespondenceCostValue_.clear();
            mapLimits_.setLimits(0,0);
            pose_.Zero();
            laserCount_ = 0;
            scale_.setEmpty();
            poseOffset_.Zero();
            finished_ = false;
            
            
        }

        void initialTable();

        void growLimitsIfNeeded(const Eigen::AlignedBox2i scale);

        void finish();

        void setFinish(){finished_ = true;}

        inline void addHit(int x,int y);

        inline void addMiss(int x,int y);

        template <typename T>
        inline void addHit(T xy);

        template <typename T>
        inline void addMiss(T xy);

        bool empty(){return CorrespondenceCostValue_.empty();}

        inline void setLaserCount(int i){laserCount_ = i;}
        
        inline const float getProbability(uint16_t x, uint16_t y)const{
            return 1-ProbabilityTable_[CorrespondenceCostValue_[mapLimits_.getCellIndex(x,y)]];
        }

        const int getLaserCount()const {return laserCount_;}

        const MapLimits & getLimits()const {return mapLimits_;}

        const Eigen::AlignedBox2i & getBoxScale() const {return scale_;}

        const std::vector<uint16_t> & getCorrespendenceValueTable()const {return CorrespondenceCostValue_;}


        /**
         * @brief 以下属于概率内部运算
         */
        
        static void checkInitialization();
        
        static void initial();
        
      private:

        static void fetchParameters();

        static void ComputeValueToProbabilityAndCorrespondCostChart();

        static void ComputetoApplyProbabilityChart();

        static void ComputetoApplyCorrespondenceCostChart();

        static float Odd (float probabilityValue) {return probabilityValue/(1.f-probabilityValue);}

        static float ProbabilityFromOdd (float odds) {return odds/(1.f +odds);}

        static float ProbabilityToCorrespondenceCost(const float probability) {return 1.f - probability;}

        static float CorrespondenceCostToProbability(const float correspondence_cost) {return 1.f - correspondence_cost;}

        static float ClampProbability(const float probability);

        static uint16_t ProbabilityToValue(const float probability);

        static uint16_t CorrespondenceCostToValue( const float correspondence_cost);

        static const ProbabilityParameters & getProbabilityParameters(){return probabilityparameter_;}

        static uint16_t getkUpdateMaker()
        {
            static uint16_t kUpdateMaker = uint16_t(1)<<15;
            return kUpdateMaker;
        }


        std::vector<uint16_t> CorrespondenceCostValue_;

        int seq_;
        int laserCount_;
        bool finished_;
        MapLimits mapLimits_;
        Eigen::Vector3f pose_;
        Eigen::AlignedBox2i scale_;
        Eigen::Vector2i poseOffset_;
        

        static bool initializationFlag_ ;
            
        static std::string paramFile_;
        static ProbabilityParameters probabilityparameter_;
        static std::vector<uint16_t> hitAddChartProbability_;
        static std::vector<uint16_t> missAddChartProbability_;
        static std::vector<uint16_t> hitAddChartCorrespondenceCost_;
        static std::vector<uint16_t> missAddChartCorrespondenceCost_;
        static std::vector<float>    ProbabilityTable_;

    };

    template <typename T>
    void ProbabilityGridMap::addHit(T xy){
        if(CorrespondenceCostValue_[mapLimits_.getCellIndex(xy(0),xy(1))]<getkUpdateMaker())
        CorrespondenceCostValue_[mapLimits_.getCellIndex(xy(0),xy(1))] = 
        hitAddChartCorrespondenceCost_[CorrespondenceCostValue_[mapLimits_.getCellIndex(xy(0),xy(1))]];
    }

    template <typename T>
    void ProbabilityGridMap::addMiss(T xy){
        if(CorrespondenceCostValue_[mapLimits_.getCellIndex(xy(0),xy(1))]<getkUpdateMaker())
        CorrespondenceCostValue_[mapLimits_.getCellIndex(xy(0),xy(1))] = 
        missAddChartCorrespondenceCost_[CorrespondenceCostValue_[mapLimits_.getCellIndex(xy(0),xy(1))]];
    }

}; // namespace LwlSLAM
