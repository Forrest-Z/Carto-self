/*
 * @Author: Liu Weilong
 * @Date: 2020-10-02 11:41:15
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-10-06 11:07:21
 * @Description: rosbag_io 进行rosbag 数据读取
 */

#include <iostream>
#include <string>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace LwlSLAM
{

class RosbagIn
{
    public:
    explicit  RosbagIn(const std::string & bag_name)
    {
        std::cout<<"open the rosbag "<<bag_name.c_str()<<std::endl;
        try
        {bag_.open(bag_name.c_str(),rosbag::bagmode::Read);}
        catch(...)
        {
            std::cerr<<" there is something wrong with the file open"<<std::endl;
        }
    }

    template <typename T>
    void FetchInfo(const std::string & topic_name,unsigned int size, std::vector<T> & output)
    {
        std::vector<std::string> topics {topic_name.c_str()};
        rosbag::View view(bag_,rosbag::TopicQuery(topics));
        unsigned int count = 1;
        
        output.clear();
        output.reserve(size);

        for(auto & element:view)
        {
            typename T::ConstPtr ptr_instaniate;
            ptr_instaniate = element.instantiate<T>();
            if(ptr_instaniate!=NULL)
            {
                output.push_back(*ptr_instaniate);
                count++;
                if(count>size)
                {
                    break;
                }
            }
        }

        return;
    }

    ~RosbagIn(){
        bag_.close();
    }

    private:
    rosbag::Bag bag_;

};
}