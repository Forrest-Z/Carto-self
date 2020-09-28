/*
 * @Author: Liu Weilong
 * @Date: 2020-09-28 07:29:17
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-09-29 07:07:16
 * @Description: Linear_Estimator 测试函数
 */


#include "gtest/gtest.h"

#include "tools/pose_extrapolator/linear_estimator.hpp"


TEST(LinearEstimatorTest,pipeline_test)
{
    LwlSLAM::LinearEstimator estimator_;
    
    LwlSLAM::PoseInfo tmp;
    LwlSLAM::PoseInfo predict;
    LwlSLAM::PoseInfo predUpdate;
    for(int i= 0;i<10;i++)
    {
        tmp.pose = Eigen::Vector3f(float(i)*3-0.5,float(i)*3-0.5,float(i)*3-0.5);
        tmp.sec = i*5;
        tmp.nsec = i*5;

        predict = estimator_.predictPose(tmp.sec,tmp.nsec);
        
        if(estimator_.getArraySize()<2)
        predict = LwlSLAM::PoseInfo();

        if(estimator_.getArraySize()<2)
        CHECK_EQ(predict.pose,Eigen::Vector3f(0,0,0));
        else
        CHECK_EQ(predict.pose,Eigen::Vector3f(float(i)*3,float(i)*3,float(i)*3));            

        tmp.pose = tmp.pose + Eigen::Vector3f(0.5,0.5,0.5);
        estimator_.insertNewPose(tmp);
    }
}