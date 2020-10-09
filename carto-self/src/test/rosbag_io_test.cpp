/*
 * @Author: Liu Weilong
 * @Date: 2020-10-06 10:57:31
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-10-06 11:06:42
 * @Description: 用于测试rosabg_io
 */
#include <vector>

#include "tools/io/rosbag_io.hpp"
#include "gtest/gtest.h"
#include "sensor_msgs/LaserScan.h"

TEST(ROSBAG_IO,GetInfoFromBag)
{
    LwlSLAM::RosbagIn bag("/home/lwl/workspace/PublicData/lidar2d/test.bag");
    std::vector<sensor_msgs::LaserScan> result_;
    bag.FetchInfo("horizontal_laser_2d",1000,result_);
    EXPECT_EQ(result_.size(),1000);
    EXPECT_NE(result_.back().ranges.size(),0);
    EXPECT_NE(result_.front().ranges.size(),0);
}