/*
 * @Author: Liu Weilong
 * @Date: 2020-09-04 15:02:56
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-10-03 17:12:26
 * @Description: 定义GridMapGenerator2d 的成员函数
 */


#include "tools/submap_generator/2d_gridmap_generator.hpp"

namespace LwlSLAM
{
    void GridMapGenerator2d::pipeline(sensor_msgs::LaserScan& laserInfo,Eigen::Vector3f pose)
    {
        if(!initialFlag_)
        {
            initial();
            initialFlag_ = true;
        }

        dataPrepare(laserInfo,pose);

        drawLaserPipeLine();
        
    };

    void GridMapGenerator2d::initial(){
        
        // 参数读取
        // TODO filename改为读取 而不是直接的赋值
        
        YAML::Node config= YAML::LoadFile(configFile_.c_str());
        
        submapCapacity_ = config["SubmapParams"]["Capacity"].as<int>();

        checkCapacity_ = config["SubmapParams"]["CheckCapacity"].as<int>();

        // 这个数值直接控制分辨率
        discreteInterval_ = config["SubmapParams"]["disInterval"].as<float>();

        submapPool_.clear();
    };

    void GridMapGenerator2d::dataPrepare(sensor_msgs::LaserScan& laserInfo,Eigen::Vector3f pose){

        CHECK(tmpMidProductXYZ_.empty());
        
        CHECK(tmpMidProductDiscretion_.empty());

        Range2XYZ(laserInfo,tmpMidProductXYZ_,pose);

        Discretize(tmpMidProductXYZ_,tmpMidProductDiscretion_,discreteInterval_);

        Discretize(pose,tmpMidProductPose_,discreteInterval_);
    };

    void GridMapGenerator2d::drawLaserPipeLine(){
        
        auto drawType = checkStatus();

        // 添加新的Submap
        if( (drawType == NoDrawingBefore)||
        (drawType == OneSubmapComplete) )
        submapPool_.push_back(ProbabilityGridMap(submapPool_.size()+1));
        
        Eigen::AlignedBox2i scale;
        scale.setEmpty();
        findMinRect(scale);
        
        CHECK(!submapPool_.empty());
        auto iterp = submapPool_.end();
        int count = submapPool_.size();
        iterp--;
        while(count!=0)
        {
            iterp->growLimitsIfNeeded(scale);

            castRays(*iterp);

            finish(*iterp);
            
            count--;
        }
    }

    GridMapGenerator2d::DrawStatus GridMapGenerator2d::checkStatus()
    {
        if(submapPool_.empty())
        return NoDrawingBefore;
        if(submapPool_.back().getLaserCount() == checkCapacity_)
        return OneSubmapComplete;
        return CommonDraw;
    }

    void GridMapGenerator2d::findMinRect(Eigen::AlignedBox2i & scale)
    {
        
        Eigen::Array2i minXY(tmpMidProductDiscretion_[0]);
        Eigen::Array2i maxXY(tmpMidProductDiscretion_[0]);

        for (auto& element : tmpMidProductDiscretion_)
        {
            minXY = minXY.min(element);
            maxXY = maxXY.max(element);
        }
        
        scale.extend(Eigen::Vector2i(tmpMidProductPose_));
        scale.extend(Eigen::Vector2i(minXY));
        scale.extend(Eigen::Vector2i(maxXY));
    }

    void GridMapGenerator2d::castRays(ProbabilityGridMap & submap)
    {
        Eigen::Vector2i bottomLeft = submap.getBoxScale().min();

        Eigen::Array2i minValue(bottomLeft);
        
        for (auto & end : tmpMidProductDiscretion_)
        {
            submap.addHit(end-minValue);
        }
        
        for (auto & end :tmpMidProductDiscretion_)
        {
            castRay(tmpMidProductPose_-minValue,end-minValue,submap);
        }
    }

    void GridMapGenerator2d::finish(ProbabilityGridMap & submap)
    {
        submap.finish();
        if (submap.getLaserCount() == submapCapacity_)
        {
            submap.setFinish();
        }
    }
    
    void GridMapGenerator2d::castRay(const Eigen::Array2i origin, const Eigen::Array2i end,ProbabilityGridMap& submap)
    {
    
        submap.addMiss(origin);

        if (origin(0)>end(0))
        {
            castRay(end,origin,submap);
            return;
        }

        if (end(0) == origin(0))
        {
            int temp = end(1) - origin(1);
            if (temp == 0)
                return ;
            int signal = temp / abs(temp);
            for (int i = 0; i <= abs(temp); i++)
            {
                submap.addMiss(Eigen::Array2i(origin(0), origin(1) + i * signal));
            }
            return;
        }


        // TODO float 计算次数过多，之后需要换成int 的整数计算
        // 对于64 位的系统，float 和int 的计算基本一致
        // 对于32 位的系统，float 和int 的计算会有很大的差别
        float ratio = float(end(1)-origin(1))/float(end(0) - origin(0));
        float length_x = 0.5;
        float length_y = ratio/2;

        while(length_x<(end(0)-origin(0)))
        {
            float coordinate_x = std::round(length_x - 1e-7);
            int sub_y = std::round(length_y - ratio/2);
            int up_y = std::round(length_y);

            for(;sub_y <= up_y; sub_y++)
            {
                submap.addMiss(Eigen::Array2i(std::round(coordinate_x + origin(0)),sub_y + origin(1)));
            }
            length_x += 0.5;
            length_y += ratio/2;
        }
        return ;
    }
} // namespace LwlSLAM
