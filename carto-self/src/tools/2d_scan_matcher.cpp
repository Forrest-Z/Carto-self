/*
 * @Author: Liu Weilong
 * @Date: 2020-09-04 15:04:02
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-09-10 16:44:41
 * @Description: CorrelativeScanMatcher 函数实现
 */
#pragma once
#include "2d_scan_matcher.hpp"
#include "common/common.hpp"

namespace LwlSLAM
{
    auto CorrelativeScanMatcher2d::pipeline(const ProbabilityGridMap & submap, 
    const Eigen::Vector3f & posePrediction, const sensor_msgs::LaserScan & laserInfo){
        if(!initialFlag_)
        {
            initial(laserInfo);
            initialFlag_ = true;
        }

        dataPrepare();

        generateRotationDiscreteScans(laserInfo,posePrediction);

        generateTranslateCandidate();

        auto result = scoreAllCandidates(submap,posePrediction);

        clearMidProduct();

        return result;
    }

    void CorrelativeScanMatcher2d::initial(const sensor_msgs::LaserScan & laserInfo)
    {
        YAML::Node config = YAML::LoadFile(configFile_.c_str());
        usingEffectiveMaxRange_ = config["CorrelativeScanMatcher"]["UsingEffectiveMaxRagne"].as<bool>();
        searchWindowParams_.pixelSize = config["CorrelativeScanMatcher"]["PixelSize"].as<float>();
        searchWindowParams_.translationWindowSize = config["CorrelativeScanMatcher"]["TranslationWindowSize"].as<float>();
        searchWindowParams_.rotationWindowSize = config["CorrelativeScanMatcher"]["RotationWindowSize"].as<float>();

        if(usingEffectiveMaxRange_)
        searchWindowParams_.predictEffectiveMaxRange = config["CorrelativeScanMatcher"]["EffectiveMaxRange"].as<float>();

        generateSearchWindow(laserInfo);
    }

    void CorrelativeScanMatcher2d::generateSearchWindow(const sensor_msgs::LaserScan & laserInfo)
    {
        searchWindowParams_.rotationLowerBound = -1 * abs(searchWindowParams_.rotationWindowSize);
        searchWindowParams_.translationLowerBound = -1 * abs(searchWindowParams_.translationWindowSize);
        searchWindowParams_.translationStepNum = std::ceil(2 * searchWindowParams_.translationWindowSize
        / searchWindowParams_.pixelSize);
        searchWindowParams_.translationStepSize = searchWindowParams_.pixelSize;

        float max_range = 0;
        auto pixelSize = searchWindowParams_.pixelSize;

        if(usingEffectiveMaxRange_)
        {
            max_range = usingEffectiveMaxRange_;
        }
        else{
            // TODO 还没有写，目前还是先使用预测的EffectiveMaxRange
        }  

        // TODO 此处存疑 theta 原版也就是 CSM.cpp 好像是有问题的，少开了一次方。所以现在加上了一个sqrt
        searchWindowParams_.rotationStepSize = std::acos(sqrt(1. - (pixelSize) * (pixelSize) / (1.5 * max_range * max_range)));
        searchWindowParams_.rotationStepNum = 2 * ceil(abs(searchWindowParams_.rotationWindowSize) /
        searchWindowParams_.rotationStepSize);
    }

    void CorrelativeScanMatcher2d::generateRotationDiscreteScans(const sensor_msgs::LaserScan & laserInfo,
    const Eigen::Vector3f & posePrediction)
    {
        CHECK(searchWindowParams_.rotationStepNum >0);
        CHECK(tmpMidProductDiscreteScans_.size()==0);
        CHECK(tmpMidProductAngleOffset_.size()==0);

        std::vector<Eigen::Vector3f> tmpLaserXYZ;
        std::vector<Eigen::Array2i> tmpDiscreteLaserXYZ;
        
        tmpLaserXYZ.reserve(laserInfo.ranges.size());
        tmpDiscreteLaserXYZ.reserve(laserInfo.ranges.size());
        tmpMidProductDiscreteScans_.reserve(searchWindowParams_.rotationStepNum);
        tmpMidProductAngleOffset_.reserve(searchWindowParams_.rotationStepNum);

        float initialAngle = searchWindowParams_.rotationLowerBound;
        float deltaAngle = searchWindowParams_.rotationStepSize;

        for(int i = 0;i<searchWindowParams_.rotationStepNum;i++)
        {
            // TODO 这个地方的DIscretize 函数的模板适用性没有测试过，需要写一个单元测试进行测试
            Range2XYZ(laserInfo,tmpLaserXYZ,Eigen::Vector3f(0,0,initialAngle+i*deltaAngle));
            Discretize(tmpLaserXYZ,tmpDiscreteLaserXYZ,searchWindowParams_.pixelSize);
            tmpMidProductDiscreteScans_.push_back(tmpDiscreteLaserXYZ);
            tmpMidProductAngleOffset_.push_back(initialAngle+i*deltaAngle);
            tmpLaserXYZ.clear();
            tmpDiscreteLaserXYZ.clear();
        }

        CHECK(tmpMidProductDiscreteScans_.size() == searchWindowParams_.rotationStepNum);
        
    }

    void CorrelativeScanMatcher2d::generateTranslateCandidate()
    {
        int lower_pixel = -1*std::floor(searchWindowParams_.translationWindowSize/searchWindowParams_.pixelSize);
        int uppper_pixel = -lower_pixel;
        
        for (int cur_x = lower_pixel ; cur_x <= uppper_pixel;cur_x++ )
        {
            for (int cur_y = lower_pixel ; cur_y <= uppper_pixel;cur_y++  )
            {
                tmpMidProductTranslationCandidates_.push_back(Eigen::Array2i(cur_x,cur_y));
            }
        }
    }

    MatchResult CorrelativeScanMatcher2d::scoreAllCandidates(const ProbabilityGridMap & submap,
    Eigen::Vector3f posePrediction)
    {
        CHECK(tmpMidProductDiscreteScans_.size()!=0);
        CHECK(tmpMidProductTranslationCandidates_.size()!=0);

        std::vector<MatchResult> resultPool_;
        resultPool_.reserve(tmpMidProductDiscreteScans_.size());
        auto pScan = tmpMidProductDiscreteScans_.begin();
        for (int pcount=0 ;pcount<searchWindowParams_.rotationStepNum;pcount++)
        {   
            auto bestmatchOffset = scoreAllCandidatesForOneRotation(submap,*pScan);
            resultPool_.push_back(bestmatchOffset + posePrediction + 
            Eigen::Vector3f(0,0,tmpMidProductAngleOffset_[pcount]));
            pScan++;
        }

        std::sort(resultPool_.begin(),resultPool_.end(),[](MatchResult a,MatchResult b)
        {return a.bestScore>b.bestScore;});

        return resultPool_.front();
    }

    MatchResult CorrelativeScanMatcher2d::scoreAllCandidatesForOneRotation(const ProbabilityGridMap & submap,
    const std::vector<Eigen::Array2i> & rotationScan)
    {
        Eigen::Array2i BestCandidate(0,0);
        Eigen::Array2i offset = Eigen::Array2i(submap.getBoxScale().min());

        float bestscore = 0.f;
        double pixelSize = searchWindowParams_.pixelSize;
        int border = tmpMidProductDiscreteScans_.front().size()*0.7;
        
        for (auto & candidate: tmpMidProductTranslationCandidates_)
        {
            float score = 0.f;
            int count = 0;

            for (auto & element: rotationScan)
            {
                Eigen::Array2i temp = element + candidate -offset;

                if (submap.getLimits().contain(temp))
                {
                    // score += (1.f - PGMBuilder::ValueToCorrespondenceCost[ProbabilityTable[int(temp(1) * maplimits_.scaleXY(0)) + temp(0)]]);
                    score +=submap.getProbability(temp(0),temp(1));
                    count++;
                }
                else
                continue;
            }
            
            if (count < border)
                continue;

                score = score / count;
            if (bestscore < score)
            {
                bestscore = score;
                BestCandidate = candidate;
            }
                score = 0;
        }

        return MatchResult{bestscore,Eigen::Vector3f(BestCandidate(0)*pixelSize,BestCandidate(1)*pixelSize,0)};
    }

    void CorrelativeScanMatcher2d::clearMidProduct()
    {
        tmpMidProductDiscreteScans_.clear();
        tmpMidProductTranslationCandidates_.clear();
        tmpMidProductAngleOffset_.clear();
    }
    
} // namespace LwlSLAM
