/*
 * @Author: Liu Weilong
 * @Date: 2020-09-04 15:00:47
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-09-07 13:33:54
 * @Description: 用于启动 LocalMatcher 这个节点
 */

// 为了防止 log 文件出问题 所以先把log 的初始化写在这里

#include "local_matcher_pipeline.hpp"
#include "ros/ros.h"
#include "glog/logging.h"

int main(int argc, char** argv)
{
    FLAGS_alsologtostderr =1;
    google::InitGoogleLogging(argv[0]);
    google::SetLogDestination(google::GLOG_WARNING,"/home/lwl/workspace/cartographer-self-ros/log/local_matcher_log");

    ros::init(argc,argv,"local_matcher_node");


    

    return 0;
}