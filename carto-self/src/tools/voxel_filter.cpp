/*
 * @Author: Liu Weilong 
 * @Date: 2020-09-02 11:00:00 
 * @Last Modified by: Liu Weilong
 * @Last Modified time: 2020-09-04 16:45:16
 */


#include "voxel_filter.hpp"

namespace LwlSLAM
{

    void VoxelFilter::pipeline(const SensorType & Input,SensorType & Output) 
    {
        dataPrepare(Input);
        downSample(Input,Output);
        clearMidProduct();
    }

    void VoxelFilter::dataPrepare(const SensorType & Input)
    {
        float initAngle = Input.angle_min;
        float increAngle = Input.angle_increment;
        
        auto initRot = Sophus::SO3<float>::rotZ(initAngle).matrix();
        auto increRot = Sophus::SO3<float>::rotZ(increAngle).matrix();

        tmpMidProductRawPoint_.reserve(Input.ranges.size());
        tmpMidProductSparcePoint_.resize(tmpMidProductRawPoint_.size());

        auto changeRot = initRot;

        for(auto range:Input.ranges)
        {
            tmpMidProductRawPoint_.push_back(changeRot*Eigen::Vector3f(range,0,0));
            changeRot = changeRot*increRot;
        }
    }
    void VoxelFilter::downSample(const SensorType & Input,SensorType & Output) 
    {

        // downSample 主体函数
        using KeyType = std::bitset<64*2>;

        Output = Input;

        std::unordered_set<KeyType> pointSet;

        int count =0;

        for(auto PointXYZ:tmpMidProductRawPoint_)
        {
            int x_t = std::ceil(PointXYZ.x()/gridSize_);
            int y_t = std::ceil(PointXYZ.y()/gridSize_);

            
            KeyType x_bit = static_cast<uint64_t>(x_t);
            KeyType y_bit = static_cast<uint64_t>(y_t);

            KeyType result_ ((x_bit<<64)|y_bit);
            
            // std::cout<<"------------------------------"<<std::endl;
            // std::cout<< x_bit.to_string()<<std::endl;
            // std::cout<< y_bit.to_string()<<std::endl;
            // std::cout<< result_.to_string()<<std::endl;
            // std::cout<<"------------------------------"<<std::endl;

            auto it_inserted = pointSet.insert(result_);
            if(it_inserted.second)
            {
                tmpMidProductSparcePoint_.push_back(PointXYZ);
            }
            else
            {
                Output.ranges[count] = 0;
            }
            count++;
        }

    }

}