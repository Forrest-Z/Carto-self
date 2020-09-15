/*
 * @Author: Liu Weilong 
 * @Date: 2020-08-26 05:08:55 
 * @Last Modified by: Liu Weilong
 * @Last Modified time: 2020-09-04 16:12:46
 */


#include "data_processor_pipeline.hpp"

namespace LwlSLAM
{

    void RawDataProcessor::pipeline()
    {
        
        if(!initialFlag_)
        {
            initial();
        }

        while(ros::ok()&&initialFlag_)
        {
            if(getCurrentLaserScan())
            {
                // TODO 第二阶段添加
                //deskew();

                downSample();

                pubInfo();

                if(midProductSave_)
                {
                    midProductSave();
                }
                // TODO : 第二阶段使用
                clearMidProduct();
            }
            else
            {
                ros::spinOnce();
            }
        }
    }

    void RawDataProcessor::initial()
    {
        std::string laserDataName,imuInfoName;

        nh_.param<std::string>("laser_data_name",laserDataName,"horizontal_laser_2d");
        nh_.param<std::string>("imu_info_name",imuInfoName,"imu");
        nh_.param<bool>("SaveOrNot",midProductSave_,false);
        nh_.param<bool>("UsingImu",usingIMU_,false);
        nh_.param<float>("voxelFilterSize",voxelFilterSize_1,0.05);
        // downSampler 
        downSampler_ = VoxelFilter(0.05);

        // 之前callback函数的参数列表是空的，所以在subscribe函数内部使用boost::bind 绑定的时候，会有来自_1 的报错
        rawLaserDataSub_ = nh_.subscribe(laserDataName.c_str(),5,&RawDataProcessor::laserDataSubCallback,this);
        rawImuInfoSub_ = nh_.subscribe(imuInfoName.c_str(),5,&RawDataProcessor::imuInfoSubCallback,this);
        
        sparseLaserDataPub_ = nh_.advertise<sensor_msgs::LaserScan>("sparse_laser_data",5);
        rawLaserDataPub_ = nh_.advertise<sensor_msgs::LaserScan>("raw_laser_data",5);

        // imuInfoPub_ 第一阶段不进行定义
        initialFlag_ = true;

        ROS_INFO("Initialization Complete!");


    }

    bool RawDataProcessor::getCurrentLaserScan()
    {
        // TODO :第二阶段再来添加imu
        // auto laser_iter_ = laserDataPool_.begin();
        // auto imu_iter_ = imuInfoPool_.begin();
        
        // // 条件检测：保证imu的最早的时间比Laser时间早 且 两个Pool 有数值

        // if(laserDataPool_.empty())
        // return false;

        // if(imuInfoPool_.empty()&&usingIMU_)
        // {
        //     return false;
        // }

        // auto laserTime_ = (*laser_iter_).get()->header.stamp.sec;
        // auto imuTime_ = (*imu_iter_).get()->header.stamp.sec;
        // if(imuTime_>laserTime_)
        // {
        //     laserDataPool_.pop_front();
        //     return false;
        // }
        
        // // 去掉最早LaserScan 之前时间的IMU数值

        // if(usingIMU_)
        // {
        //     while(imuTime_<laserTime_)
        //     {
        //         auto imuTime_t = (*imu_iter_++).get()->header.stamp.sec;
        //         if (imuTime_t>=laserTime_) 
        //         break;
        //         imuInfoPool_.pop_front();
        //     }
        // }
        // // 如果LaserScan 的Period 内的imu已经拿全 就取出这个段imu和LaserScan 进行deskew

#ifdef ROS_BAG_INPUT        
        rosBagIO(currentLaserScan_);
        return true;
#else
        if(laserDataPool_.empty()){
            return false;
        }
        else{
            currentLaserScan_ = laserDataPool_.front();
            laserDataPool_.pop_front();
            return true;
        }
#endif // ROS_BAG_INPUT
    }

    void RawDataProcessor::downSample()
    {
        sparseLaserScan_ ;
        downSampler_.pipeline(currentLaserScan_,sparseLaserScan_);
    }

    void RawDataProcessor::pubInfo(){
        sparseLaserDataPub_.publish(sparseLaserScan_);
        rawLaserDataPub_.publish(currentLaserScan_);
        ROS_DEBUG("Raw Data Processor Publish a Cluster of info!");
        ROS_DEBUG("The pubed sparse laser info seq is % d",sparseLaserScan_.header.seq);
    }
    
    void RawDataProcessor::midProductSave(){
         
        
    }

    void RawDataProcessor::clearMidProduct(){
        // TODO ：操作冗余，downSampler_内部会进行一次 clear 第二阶段去除
        // downSampler_.clearMidProduct();
    }
#ifdef ROS_BAG_INPUT
        void RawDataProcessor::rosBagIO(sensor_msgs::LaserScan & output)
    {
        static rosbag::Bag bag_;
        static bool bagOpen_ ;
        if(!initialBag_)
        {
            bag_.open("/home/lwl/workspace/PublicData/lidar2d/test.bag",rosbag::bagmode::Read);
            bagOpen_ = true;
            initialBag_ = true;
        }
        
        if(bagOpen_)
        {
        static std::vector<std::string> topics{"horizontal_laser_2d"};
        
        static rosbag::View view(bag_,rosbag::TopicQuery(topics));

        static auto iter_ = view.begin();

        
        output = *(iter_)->instantiate<sensor_msgs::LaserScan>();

        iter_++;
        if((iter_)==view.end())
        {
            bag_.close();
            bagOpen_ = false;
        }
        }
    }
#endif //ROS_BAG_INPUT
}

