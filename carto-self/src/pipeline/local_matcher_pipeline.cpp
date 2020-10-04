/*
 * @Author: Liu Weilong 
 * @Date: 2020-09-04 15:20:33 
 * @Last Modified by: Liu Weilong
 * @Last Modified time: 2020-09-05 14:06:50
 */

#include "local_matcher_pipeline.hpp"

namespace LwlSLAM
{

    void LidarMatcher::pipeline()
    {
        if(!initialFlag_)
        {    
            // 各种参数的初始化
            initial();
            initialFlag_ = true;
        }

        while(ros::ok()&&initialFlag_)
        {
            if(fetchInfo())
            {
                pretreatData();

                predictPose();
                
                match();

                generateMap();
            }
        }
    }

    void LidarMatcher::initial()
    {
        std::string laserTopic,imuTopic;
        nh_.param("laserTopic",laserTopic,std::string(""));
        nh_.param("imuTopic",imuTopic,std::string(""));
        laserInfoSub_ = nh_.subscribe(laserTopic.c_str(),
        300,&LidarMatcher::laserinfoCb,this);
        ImuInfoSub_ = nh_.subscribe(imuTopic,
        1000,&LidarMatcher::imuinfoCb,this);

        laserScanPool_.clear();
        imuInfoPool_.clear();
    }

    // 不融合imu 的情况下 目前只提取laserinfo
    bool LidarMatcher::fetchInfo()
    {
        currentScan_ = laserScanPool_.front();
        laserScanPool_.pop_front();
        return true;
    }

    void LidarMatcher::pretreatData()
    {

    }
    
    void LidarMatcher::predictPose()
    {
        // TODO 1. 这里的时间使用肯定是有问题的
        //      2. 边界情况的考虑
        if(poseExtrapolator_.getArraySize() < 2)
        predictPose_ = PoseInfo();

        predictPose_ =  poseExtrapolator_.predictPose(currentScan_.header.stamp.sec
        ,currentScan_.header.stamp.nsec);
    }

    void LidarMatcher::generateMap()
    {      
        submapGenerator_.pipeline(currentScan_,predictPoseUpdate_.pose); 
    }

    void LidarMatcher::match()
    {
        if(submapGenerator_.getPoolSize() == 0)
        {
            predictPoseUpdate_ = predictPose_;
            return ;
        }
        matcher_.pipeline(submapGenerator_.getCurrentMap(),predictPose_.pose,
        currentScan_);
    }

    // TODO 这个地方一定会限制算法，因为会进行一次赋值拷贝
    //      而且没有一个mutex 在多线程的时候 一定是不安全的
    void LidarMatcher::laserinfoCb(const sensor_msgs::LaserScan::ConstPtr  msg)
    {
        laserScanPool_.push_back(*msg);
    }
    
    void LidarMatcher::imuinfoCb(const sensor_msgs::ImuConstPtr  msg)
    {
        imuInfoPool_.push_back(*msg);
    }
}