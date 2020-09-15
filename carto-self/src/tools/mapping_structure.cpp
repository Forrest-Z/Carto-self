/*
 * @Author: Liu Weilong
 * @Date: 2020-09-07 10:40:54
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-09-10 08:54:49
 * @Description: 用于概率栅格地图的函数定义
 */

#include "mapping_structure.hpp"

namespace LwlSLAM
{
    /**
     * @brief 以下定义 MapLimits
     */

    /**
     * @brief 这里的是 二维坐标转为1维坐标
    */
    int MapLimits::getCellIndex(int x, int y)const {
        return y*scaleXY_.x()+x;
    }

    /**
     * @brief 判断xy二维坐标是不是在submap内 
    */
    bool MapLimits::contain(int x,int y) const{
        return (x<scaleXY_.x())&&(y<scaleXY_.y())&&(x>=0)&&(y>=0);
    }

    template<typename T>
    bool MapLimits::contain(T xy) const{
        int x = xy(0);
        int y = xy(1);
        return (x<scaleXY_.x())&&(y<scaleXY_.y())&&(x>=0)&&(y>=0);
    }

    void MapLimits::setLimits(int x,int y){
        CHECK(x>0);
        CHECK(y>0);
        scaleXY_ = Eigen::Vector2i(x,y);
    }

    /**
     * @brief 以下定义ProbabilityGridMap
    */
    uint16_t kUpdateMaker = uint16_t(1)<<15;
    bool ProbabilityGridMap::initializationFlag_;
    std::string ProbabilityGridMap::paramFile_;


    ProbabilityParameters ProbabilityGridMap::probabilityparameter_;
    std::vector<uint16_t> ProbabilityGridMap::hitAddChartProbability_;
    std::vector<uint16_t> ProbabilityGridMap::missAddChartProbability_;
    std::vector<uint16_t> ProbabilityGridMap::hitAddChartCorrespondenceCost_;
    std::vector<uint16_t> ProbabilityGridMap::missAddChartCorrespondenceCost_;
    std::vector<float>    ProbabilityGridMap::ProbabilityTable_;

    void ProbabilityGridMap::initialTable(){
        CHECK(!scale_.isEmpty());
        
        auto BottomLeft = scale_.min();
        auto TopRight = scale_.max();

        int capacity = (TopRight(0)-BottomLeft(0) + 1)*(TopRight(1) - BottomLeft(1) + 1);

        uint16_t tmpValue = CorrespondenceCostToValue(0.5);

        mapLimits_.setLimits(TopRight(0)-BottomLeft(0) + 1,TopRight(1) - BottomLeft(1) + 1);
        
        CorrespondenceCostValue_.resize(capacity,tmpValue);
    }

    void ProbabilityGridMap::growLimitsIfNeeded(const Eigen::AlignedBox2i scale)
    {
        if(CorrespondenceCostValue_.empty())
        {
            scale_ = scale;       
            initialTable();
            return;
        }    

        // 检查是否存在超过限制的情况
        Eigen::AlignedBox2i tmpScale(scale_);

        Eigen::Vector2i minExtendArea,maxExtendArea,margin(50,50);

        minExtendArea.Zero();
        maxExtendArea.Zero();

        tmpScale.extend(scale.min());
        tmpScale.extend(scale.max());

        minExtendArea = (scale_.min() - tmpScale.min());
        maxExtendArea = (tmpScale.max() - scale_.max());

        if (minExtendArea.maxCoeff()==0 && maxExtendArea.maxCoeff()==0)
        return;

        // 如果有增长就进行拓展
        MapLimits newLimits;
        std::vector<uint16_t> newTable;
        Eigen::Vector2i newScale;

        scale_.extend(scale_.min()-minExtendArea-margin);
        scale_.extend(scale_.max()+maxExtendArea+margin);

        newScale = Eigen::Vector2i(scale_.max()-scale_.min()+Eigen::Vector2i(1,1));

        newLimits.setLimits(newScale(0),newScale(1));

        newTable.resize(newScale(0)*newScale(1),CorrespondenceCostToValue(0.5));
        
        uint32_t stride = newLimits.getLimits()(0);
        uint32_t offsetBottom = stride * minExtendArea(1);
        uint32_t offsetLeftMargin = minExtendArea(0)+margin(0);
        uint32_t oldStride = mapLimits_.getLimits()(0);

        // Update the whole localMap
        for(uint32_t row = 0;row <mapLimits_.getLimits()(1);row++)
        {
            for (uint32_t col = 0; col<mapLimits_.getLimits()(0);col++)
            {
                uint32_t temp_x =offsetLeftMargin + col;
                newTable[offsetBottom + row*stride + temp_x ] = CorrespondenceCostValue_[row*oldStride + col];
            }
        }

        CorrespondenceCostValue_ = newTable;
        mapLimits_ = newLimits;
    }

    void ProbabilityGridMap::finish()
    {
        laserCount_ += 1;
        for (auto & element: CorrespondenceCostValue_)
        {
        if (element >kUpdateMaker)
            element -= kUpdateMaker;
        }
    }

    void ProbabilityGridMap::addHit(int x,int y){
        
        CHECK(mapLimits_.contain(x-scale_.min()(0),y-scale_.min()(1)));
        if(CorrespondenceCostValue_[mapLimits_.getCellIndex(x,y)]<kUpdateMaker)
        CorrespondenceCostValue_[mapLimits_.getCellIndex(x,y)] = 
        hitAddChartCorrespondenceCost_[CorrespondenceCostValue_[mapLimits_.getCellIndex(x,y)]];
    }

    void ProbabilityGridMap::addMiss(int x,int y){
        CHECK(mapLimits_.contain(x-scale_.min()(0),y-scale_.min()(1)));
        if(CorrespondenceCostValue_[mapLimits_.getCellIndex(x,y)]<kUpdateMaker)
        CorrespondenceCostValue_[mapLimits_.getCellIndex(x,y)] = 
        missAddChartCorrespondenceCost_[CorrespondenceCostValue_[mapLimits_.getCellIndex(x,y)]];
    }

    template <typename T>
    void ProbabilityGridMap::addHit(T xy){
        if(CorrespondenceCostValue_[mapLimits_.getCellIndex(xy(0),xy(1))]<kUpdateMaker)
        CorrespondenceCostValue_[mapLimits_.getCellIndex(xy(0),xy(1))] = 
        hitAddChartCorrespondenceCost_[CorrespondenceCostValue_[mapLimits_.getCellIndex(xy(0),xy(1))]];
    }

    template <typename T>
    void ProbabilityGridMap::addMiss(T xy){
        if(CorrespondenceCostValue_[mapLimits_.getCellIndex(xy(0),xy(1))]<kUpdateMaker)
        CorrespondenceCostValue_[mapLimits_.getCellIndex(xy(0),xy(1))] = 
        missAddChartCorrespondenceCost_[CorrespondenceCostValue_[mapLimits_.getCellIndex(xy(0),xy(1))]];
    }

    void ProbabilityGridMap::checkInitialization(){
        CHECK(initializationFlag_==true);
    }

    void ProbabilityGridMap::initial(){
        CHECK(initializationFlag_ == false);
        CHECK(hitAddChartProbability_.size()==0);
        CHECK(missAddChartProbability_.size()==0);
        CHECK(hitAddChartCorrespondenceCost_.size()==0);
        CHECK(missAddChartProbability_.size() ==0);

        if(!initializationFlag_)
        {
            fetchParameters();

            ComputeValueToProbabilityAndCorrespondCostChart();

            ComputetoApplyProbabilityChart();
            
            ComputetoApplyCorrespondenceCostChart();
            
            initializationFlag_ = true;
        }
    }

    void ProbabilityGridMap::fetchParameters()
    {
        //TODO 目前是一个固定的位置，之后一定要进行更换
        YAML::Node config = YAML::LoadFile("/home/lwl/workspace/cartographer-self-ros/src/carto-self/config/ProbabilityParams.yaml");

        CHECK(config.size()!=0);

        probabilityparameter_.highBound = config["ProbabilityParams"]["highBound"].as<float>();
        probabilityparameter_.lowBound = config["ProbabilityParams"]["lowBound"].as<float>();
        probabilityparameter_.hitOdd = config["ProbabilityParams"]["hitOdd"].as<float>();
        probabilityparameter_.missOdd = config["ProbabilityParams"]["missOdd"].as<float>();
        probabilityparameter_.unknown = config["ProbabilityParams"]["unknown"].as<float>();
    }

    void ProbabilityGridMap::ComputeValueToProbabilityAndCorrespondCostChart(){
        ProbabilityTable_.reserve(uint16_t(1)<<15);
        float interval = probabilityparameter_.highBound - probabilityparameter_.lowBound;
        float deltaProb = interval/float(uint16_t(1)<<15);

        for(int i =0;i<(uint16_t(1)<<15);i++)
        {
            ProbabilityTable_.push_back(probabilityparameter_.lowBound + deltaProb*i);
        }
        
        CHECK(ProbabilityTable_.back()==0.9);
        CHECK(ProbabilityTable_.front()==0.1);
        
    }

    void ProbabilityGridMap::ComputetoApplyProbabilityChart(){
        hitAddChartProbability_.reserve(uint16_t(1)<<15);
        missAddChartProbability_.reserve(uint16_t(1)<<15);

        for(auto elem: ProbabilityTable_)
        {
            // hitOdd
            hitAddChartProbability_.push_back(ProbabilityToValue
            (ProbabilityFromOdd(Odd(elem)*probabilityparameter_.hitOdd))+kUpdateMaker);

            // missOdd
            missAddChartProbability_.push_back(ProbabilityToValue
            (ProbabilityFromOdd(Odd(elem)*probabilityparameter_.missOdd))+kUpdateMaker);
        }
        
        CHECK(hitAddChartProbability_.size()==(uint16_t(1)<<15));
        CHECK(hitAddChartProbability_.size()==(uint16_t(1)<<15));
        
    }

    void ProbabilityGridMap::ComputetoApplyCorrespondenceCostChart(){
        hitAddChartCorrespondenceCost_.reserve(uint16_t(1)<<15);
        missAddChartCorrespondenceCost_.reserve(uint16_t(1)<<15);

        for(auto elem:ProbabilityTable_)
        {
            //hitOdd
            hitAddChartCorrespondenceCost_.push_back(kUpdateMaker+CorrespondenceCostToValue
            (1.f-ProbabilityFromOdd(Odd(1.f-elem)*probabilityparameter_.hitOdd)));
            //missOdd
            missAddChartCorrespondenceCost_.push_back(kUpdateMaker+CorrespondenceCostToValue
            (1.f-ProbabilityFromOdd(Odd(1.f-elem)*probabilityparameter_.missOdd)));
        }
    }

    float ProbabilityGridMap::ClampProbability(const float probability){
        
        if (probability > probabilityparameter_.highBound)
            return probabilityparameter_.highBound;
        if (probability<probabilityparameter_.lowBound)
            return probabilityparameter_.lowBound;
        return probability;
    }

    uint16_t ProbabilityGridMap::ProbabilityToValue(const float probability){
    uint16_t value =
      round(
          (ClampProbability(probability) - probabilityparameter_.lowBound) *
          (32766.f / (probabilityparameter_.highBound - probabilityparameter_.lowBound))) +
      1;
      return value;
    }

    uint16_t ProbabilityGridMap::CorrespondenceCostToValue( const float correspondence_cost){
    uint16_t value =
      round(
          (ClampProbability(correspondence_cost) - probabilityparameter_.lowBound) *
          (32766.f / (probabilityparameter_.highBound - probabilityparameter_.lowBound))) +
      1;
      return value;
    }

    
} // namespace LwlSLAM


