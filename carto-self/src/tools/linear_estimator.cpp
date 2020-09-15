/*
 * @Author: Liu Weilong
 * @Date: 2020-09-11 14:18:29
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-09-11 14:54:13
 * @Description: LinearEstimator 函数定义
 */

#pragma once

#include <iostream>

#include "linear_estimator.hpp"

namespace LwlSLAM
{   
    void LinearEstimator::initial()
    {
        poseArray_.clear();
    }

    void LinearEstimator::insertNewPose(PoseInfo poseInfo_)
    {
        if(poseArray_.size()==0)
        {
            poseArray_.push_back(poseInfo_);
            return;
        }
        
        CHECK(poseArray_.back().sec<=poseInfo_.sec);
        if(poseArray_.back().sec == poseInfo_.sec)
        CHECK(poseArray_.back().nsec<poseInfo_.nsec);

        float deltaTime = 0;
        Eigen::Vector3f deltaDistance = poseInfo_.pose - poseArray_.back().pose;
        
        deltaTime += static_cast<float>(poseInfo_.sec - poseArray_.back().sec);
        deltaTime += static_cast<float>(poseInfo_.nsec - poseArray_.back().nsec)*pow10(-9);

        poseArray_.back().velocity = deltaDistance/deltaTime;
        poseArray_.push_back(poseInfo_);
    }

    PoseInfo LinearEstimator::predictPose(uint32_t sec, uint32_t nsec)
    {
        CHECK(poseArray_.back().sec<=sec);
        if(poseArray_.back().sec == sec)
        CHECK(poseArray_.back().nsec<nsec);
        
        PoseInfo tmp = poseArray_.back();
        float deltaTime = 0;
        deltaTime += static_cast<float>(sec - poseArray_.back().sec);
        deltaTime += static_cast<float>(nsec - poseArray_.back().nsec)*pow10(-9);

        tmp.sec = sec;
        tmp.nsec = nsec;
        tmp.pose = tmp.pose + tmp.velocity*deltaTime;
        tmp.velocity.Zero();
        tmp.seq =0;
        
        return tmp;

    }
}