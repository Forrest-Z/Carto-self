/*
 * @Author: Liu Weilong
 * @Date: 2020-10-06 11:09:52
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-10-10 06:55:08
 * @Description: displayer的算法流程 这里我们想要写一个非常有趣的displayer 直接使用c++ decltype 的特性来进行Pub 的生成
 */

// #include "ros/ros.h"
// #include <iostream>
// #include <vector>
// #include <pair>
// #include <functional>

// namespace LwlSLAM
// {
//     class Displayer
//     {
//         public:
//         Displayer(){}

//         template <typename T>
//         void registratePub(const std::string & topic_name, T sample,unsigned int puffer_size)
//         {
//             ros::Publisher tmp = nh_.advertise<>
//         }

//         private:
        
//         ros::NodeHandle nh_;
//         std::vector<std::pair<std::string,ros::Publisher>> pubPool_;
//         std::vector<std::pair<std::string,ros::Timer>> timerPool_;

//     }
// } // namespace LwlSLAM

