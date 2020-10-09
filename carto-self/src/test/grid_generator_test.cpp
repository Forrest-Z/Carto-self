/*
 * @Author: Liu Weilong
 * @Date: 2020-09-29 07:08:04
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-10-10 07:51:04
 * @Description: 用于girdmap_generator 进行测试
 */

#pragma once
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <iostream>
#include <vector>
#include "tools/submap_generator/2d_gridmap_generator.hpp"
#include "tools/io/rosbag_io.hpp"
#include "common/common.hpp"

namespace LwlSLAM
{
class GridMapGeneratorTest:public ::testing::Test
{
    public:

    static void SetUpTestCase(){
        std::cout<<"SETUPCASE"<<std::endl;
        rosbag_in_.FetchInfo("horizontal_laser_2d",30,LaserScanArray_);
        EXPECT_EQ(LaserScanArray_.size(),30);
    }
    static void TearDownTestCase(){}
    
  
    static std::string rosbag_file_;
    static LwlSLAM::RosbagIn rosbag_in_;
    static std::vector<sensor_msgs::LaserScan> LaserScanArray_;
};

std::string GridMapGeneratorTest::rosbag_file_("/home/lwl/workspace/PublicData/lidar2d/test.bag");
LwlSLAM::RosbagIn GridMapGeneratorTest::rosbag_in_(GridMapGeneratorTest::rosbag_file_);
std::vector<sensor_msgs::LaserScan> GridMapGeneratorTest::LaserScanArray_;



TEST_F(GridMapGeneratorTest,MapGenerator)
{
    std::string yaml_file_ = "/home/lwl/workspace/cartographer-self-ros/src/carto-self/config/SubMapGenerator2d.yaml";
    LwlSLAM::GridMapGenerator2d gmg2d(yaml_file_);
    
    gmg2d.pipeline(LaserScanArray_.front(),Eigen::Vector3f(0,0,0));
    
    // 检查地图是否增加
    CHECK_EQ(gmg2d.getPoolSize(),1);
    auto tmp_map1 = gmg2d.getCurrentMap();
    auto tmp_map2 = gmg2d.getGridMap(0);
    
    // 检查是否初始化
    tmp_map1.checkInitialization();
    
    // 检查地图更新是否成功
    auto limit = tmp_map1.getLimits();
    
    std::vector<Eigen::Vector3f> tmpXYZ_;
    Range2XYZ(LaserScanArray_.front(),tmpXYZ_,Eigen::Vector3f(0,0,0.5));
    std::vector<Eigen::Array2i> tmpDiscrete_;
    CHECK_EQ(gmg2d.getPixelSize(),0.05);
    Discretize(tmpXYZ_,tmpDiscrete_,gmg2d.getPixelSize());

    Eigen::Vector2i minValue (tmp_map1.getBoxScale().min());
    
    for(auto & element:tmpDiscrete_)
    {
        auto temp = element-Eigen::Array2i(minValue);
        EXPECT_GT(tmp_map1.getProbability(temp(0),temp(1)),0.5);
    }

    LOG(INFO)<<tmp_map1.getBoxScale().min();
}

TEST_F(GridMapGeneratorTest,MapGrow)
{
    std::string yaml_file_ = "/home/lwl/workspace/cartographer-self-ros/src/carto-self/config/SubMapGenerator2d.yaml";
    LwlSLAM::GridMapGenerator2d gmg2d(yaml_file_);
    gmg2d.pipeline(LaserScanArray_.front(),Eigen::Vector3f(0,0,0));
    gmg2d.pipeline(LaserScanArray_.front(),Eigen::Vector3f(-10,10,0));

    CHECK_EQ(gmg2d.getCurrentMap().getLaserCount(),2);
    LOG(INFO)<< "the BottomLeft of the map "<<gmg2d.getCurrentMap().getBoxScale().min();
    
    LOG(INFO)<< "the limit of the map is "<<gmg2d.getCurrentMap().getLimits().getLimits();
}

TEST_F(GridMapGeneratorTest,MapComplete)
{
    std::string yaml_file_ = "/home/lwl/workspace/cartographer-self-ros/src/carto-self/config/SubMapGenerator2d.yaml";
    LwlSLAM::GridMapGenerator2d gmg2d(yaml_file_);
    for(auto & laser_scan:LaserScanArray_)
    gmg2d.pipeline(laser_scan,Eigen::Vector3f(0,0,0));

    CHECK_EQ(gmg2d.getPoolSize(),3);
    CHECK_EQ(gmg2d.getGridMap(0).getLaserCount(),10);
    CHECK_EQ(gmg2d.getGridMap(1).getLaserCount(),10);
    CHECK_EQ(gmg2d.getGridMap(2).getLaserCount(),10);
}
}

