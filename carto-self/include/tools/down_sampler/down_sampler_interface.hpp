/*
 * @Author: Liu Weilong 
 * @Date: 2020-09-01 15:59:42 
 * @Last Modified by: Liu Weilong
 * @Last Modified time: 2020-09-02 13:26:40
 */

#pragma once 

#include <iostream>
#include <functional>


namespace LwlSLAM
{
   template<typename T1>
    class DownSamplingInterface 
    {
        public:
        DownSamplingInterface(){}
        virtual void pipeline(const T1 & Input,T1 & Output) = 0 ;
        ~DownSamplingInterface(){}
    };
} // namespace LwlSLAM 