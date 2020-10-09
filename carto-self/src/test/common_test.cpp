/*
 * @Author: Liu Weilong
 * @Date: 2020-10-02 13:39:58
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-10-10 06:12:12
 * @Description: common.hpp 测试文件
 */
#pragma once 
#include "common/common.hpp"
#include "gtest/gtest.h"
#include "tools/io/rosbag_io.hpp"
#include "Eigen/Eigen"
#include <iostream>
#include <vector>

namespace LwlSLAM
{

float kTolerance = 1e-6;

class CommonFixture:public ::testing::Test
{
    public:

    static void SetUpTestCase(){
        rosbag_in_.FetchInfo("horizontal_laser_2d",15,LaserScanArray_);
        EXPECT_EQ(LaserScanArray_.size(),15);
    }
    static void TearDownTestCase(){}
    
  
    static std::string rosbag_file_;
    static LwlSLAM::RosbagIn rosbag_in_;
    static std::vector<sensor_msgs::LaserScan> LaserScanArray_;
};

std::string CommonFixture::rosbag_file_("/home/lwl/workspace/PublicData/lidar2d/test.bag");
LwlSLAM::RosbagIn CommonFixture::rosbag_in_(CommonFixture::rosbag_file_);
std::vector<sensor_msgs::LaserScan> CommonFixture::LaserScanArray_(0);

TEST_F(CommonFixture,TransformAndDiscre)
{
    // Transform 测试
    std::vector<Eigen::Vector3f> tmpXYZ_;
    Range2XYZ(LaserScanArray_.front(),tmpXYZ_,Eigen::Vector3f(0,0,0.5));
    CHECK_EQ(tmpXYZ_.size(),LaserScanArray_.size());
    float theta = 0.5;
    EXPECT_NEAR(tmpXYZ_.front().x(),LaserScanArray_.front().ranges.front()*std::cos(theta),kTolerance);
    EXPECT_NEAR(tmpXYZ_.front().y(),LaserScanArray_.front().ranges.front()*std::sin(theta),kTolerance);

    // Discre 测试
    std::vector<Eigen::Array2i> tmpDiscrete_;
    Discretize(tmpXYZ_,tmpDiscrete_,0.05);
    
    CHECK_EQ(tmpDiscrete_.size(),tmpXYZ_.size());
}


}