/*
 * @Author: Liu Weilong
 * @Date: 2020-09-10 10:44:27
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-09-11 13:23:53
 * @Description: 用于ScanMatcher 的结构
 */
#pragma once 
#include "Eigen/Eigen"


namespace LwlSLAM
{

    struct SearchWindowParameter
    {
        int rotationStepNum;
        int translationStepNum;
        float pixelSize;
        float predictEffectiveMaxRange;
        float rotationStepSize;
        float translationStepSize;
        float translationLowerBound;
        float rotationLowerBound;
        float rotationWindowSize;
        float translationWindowSize;
    };
    
    struct MatchResult
    {
        float bestScore ;
        Eigen::Vector3f bestPose;

        MatchResult & operator+(const MatchResult rhs)
        {
            bestPose = bestPose+rhs.bestPose;
            return *this;
        }
        
        MatchResult & operator+(const Eigen::Vector3f rhs)
        {
            this->bestPose = this->bestPose + rhs;
            return *this;
        }
    };

    struct PoseInfo
    {
        PoseInfo(){
            seq = 0;
            sec = 0;
            nsec = 0;
            pose.Zero();
            velocity.Zero();
        }
        uint32_t seq;
        uint32_t sec;
        uint32_t nsec;
        Eigen::Vector3f pose;
        Eigen::Vector3f velocity;
    }
}