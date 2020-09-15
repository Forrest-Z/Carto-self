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
            initial();
            initialFlag_ = true;
        }

        while(ros::ok()&&initialFlag_)
        {
            if(fetchInfo())
            {
                pretreatData();

                predictPose();

                generateMap();
                
                match();
            }
        }
    }

    
    
}