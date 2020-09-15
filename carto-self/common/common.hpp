/*
 * @Author: Liu Weilong
 * @Date: 2020-09-08 16:07:17
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-09-10 15:19:28
 * @Description: common 文件夹用于处理一些常用的函数  range_to_xyz 主要是用于处理 从 range 到Eigen 库内容的转换
 */

#pragma once

#include <vector>
#include <functional>

#include "Eigen/Eigen"
#include "sensor_msgs/LaserScan.h"
#include "so3.hpp"

namespace LwlSLAM
{

    bool checkLimit(float max,float min, float range)
    {
        return (range<max)&&(range>min);
    };
    /**
     * @description: 用于 Range到 XYZ 的转换
     * @param input  -> LaserScan Ranges 的来源
     *        output -> 输出的XYZ 数据，为了之后可以换成 Vector3f 或者 Vector3d 所以这里用 模板来写
     *        pose   -> 这里pose是laser(不是机器人！)相对于世界坐标系的坐标
     * @return {type} 
     */
    template<typename OutputType>
    void Range2XYZ(const sensor_msgs::LaserScan & input, std::vector<OutputType> & output,
     Eigen::Vector3f pose = Eigen::Vector3f(0,0,0))
    {
        output.reserve(input.ranges.size());

        float maxRange = input.range_max;
        float minRange = input.range_min;

        float thetaIncre = input.angle_increment;
        float thetaInitial = input.angle_min + pose.z();
        
        CHECK(minRange > 0);

        int count =1 ;

        for(auto range: input.ranges)
        {
            if(checkLimit(maxRange,minRange,range))
            {   
                OutputType tmp;
                tmp.Zero();
                float x = range * cos(thetaInitial + thetaIncre * count)+pose.x();
                float y = range * sin(thetaInitial + thetaIncre * count)+pose.y();
                tmp(0)=x;
                tmp(1)=y;
                output.push_back(tmp);
                count++;
            }
        }
    };


    template <typename T>
    std::function<int(T)> integrateType(int i)
    {
        switch(i)
        { case 1:
            return std::ceil;
            break;
          case 2:
            return std::round;
            break;
          case 3:
            return std::floor;
            break;
          default:
            return std::ceil;  
        }
    }

    /**
     * @description: 这个函数主要是用于离散化 ， 为了适用于多种Eigen的数据类型，所以使用模板。但是仅适用于Vector3<Scalar>
     * @param {type} 
     * @return {type} 
     */
    template<typename Type,typename OutputType>
    void Discretize(const std::vector<Type>& input,std::vector<OutputType> & output, float interval,int type = 1)
    {
        output.reserve(input.size());

        auto integer = integrateType<double>(type);

        for(auto vector:input)
        {
            output.push_back(OutputType(integer(vector(0)/interval),integer(vector(1))/interval)));
        }
    }

    /**
     * @description:  仅适用于Eigen::Vector<Scalar> -> Matrix<2,1,Scalar>
     */
    template<typename Type,typename OutputType>
    void Discretize(const Type input, OutputType & output, float interval,int type =1)
    {
        output.Zero();

        auto integer = integrateType<double>(type);

        output(OutputType(integer(input(0)/interval),integer(input(1)/interval)));
    }
} // namespace LwlSLAM

