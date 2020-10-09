/*
 * @Author: Liu Weilong
 * @Date: 2020-09-16 06:57:39
 * @LastEditors: Liu Weilong
 * @LastEditTime: 2020-10-10 06:57:09
 * @Description: 包含CostFunction的头文件
 */

#pragma once
#include <vector>
#include "ceres/ceres.h"
#include "Eigen/Eigen"
#include "glog/logging.h"
#include "mapping_structure.hpp"
#include "common.hpp"
namespace LwlSLAM
{
    class TranslationMatchError
    {
        public:
        TranslationMatchError(const double scaleFactor,const Eigen::Vector2d initialTranslation):
        scaleFactor_(scaleFactor),initialTranslation_(initialTranslation){}
        template<typename T>
        bool operator()(const T * translation, T* residual) const
        {
            residual[0] = T(initialTranslation_(0)) - translation(0);
            residual[0] *= scaleFactor_;
            residual[1] = T(initialTranslation_(1)) - translation(1);
            residual[1] *= scaleFactor_;
            return true;
        };
        const double scaleFactor_;
        const Eigen::Vector2d initialTranslation_;
    };

    class RotationMatchError
    {
        public:
        RotationMatchError( double scaleFactor, const double initialRotation ):
        scaleFactor_(scaleFactor),initialRotation_(initialRotation){}

        template<typename T>
        bool operator()(const T* rotation, T*residual)const{
            
            
            residual[0] = initialRotation_ - rotation[0];
            residual[0] *= scaleFactor_;
            return true;
        }


        const double scaleFactor_;
        const double initialRotation_;
    };

    class OccupiedSpaceMatchError
    {
        OccupiedSpaceMatchError(float scaleFactor,
        const std::vector<Eigen::Vector2f> & laserInfo,
        const ProbabilityGridMap & submap):scaleFactor_(scaleFactor),
        laserInfo_(laserInfo),submap_(submap),gird_size_()
        {
            CHECK_NE(laserInfo_.size(),0);
            CHECK_NE(submap_.getCorrespendenceValueTable().size(),0);
        }

        // TODO 需要后期进行重新编写 ，这里可能会进行多次计算
        template <typename T>
        bool operator()(const T * const initial_pose_ ,T* residual)const {
                   
            Eigen::Rotation2D<T> rotation(initial_pose_[2]);
            Eigen::Matrix<T,2,2> rotation_matrix = rotation.toRotationMatrix();
            Eigen::Matrix<T,2,1> translation(initial_pose_[0],initial_pose_[1]);
            Eigen::Matrix<T,3,3> transfrom;
            
            transfrom<<rotation_matrix<<translation<<T(0),T(0),T(1);
            
            for(auto laserPoint:laserInfo_)
            {
                // 目前的猜测是 这里使用matrix T 是为了让Eigen 计算可以完成
                // 如果直接写计算的话是可以使用double 进行的，因为这里所有点的v 都是来于rc 自身的v == 0

                // 也就是说 这里做的是 f(x) 除了x之外所有的都是 常数值 
                Eigen::Matrix<T,3,1> pointInVector3T (T(laserPoint(0)),T(laserPoint(1)),T(1.0));
                auto transformedPoint = transfrom*pointInVector3T;
                getValue(pointInVector3T(0),pointInVector3T(1),residual);
            }
            *residual = scaleFactor_*(*residual);
            return true;
        } 

        // 对应jet的情况
        template <typename T>
        void getValue(const T r, const T c,T *residual) const
        {
            double dfdc=0,dfdr=0,f=0;
            Evaluate(r.a,c.a,&dfdc,&dfdr,&f);
            residual.a = f;
            residual.v = dfdc*c.v + dfdr*r.v;
        }

        void getValue(const double r,const double c ,double * residual) const
        {
            Evaluate(r,c,NULL,NULL,residual);
        }

        // 对应double 的情况
        void Evaluate(double r,double c,double *dfdc__ ,double *dfdr__,double*f) const
        {
            int r_ = std::floor(r/gird_size_);
            int c_ = std::floor(c/gird_size_);
            double delta_r = r/gird_size_ - r;
            double delta_c = c/gird_size_ - c;
            
            double p0,p1,p2,p3;
            double dfdc[4]{0,0,0,0};
            double fc[4]{0,0,0,0};
            for (int i = -1;i<3;i++)
            {
                p0 = 1-submap_.getCorrespendence(r_+i,c_-1);
                p1 = 1-submap_.getCorrespendence(r_+i,c_);
                p2 = 1-submap_.getCorrespendence(r_+i,c_+1);
                p3 = 1-submap_.getCorrespendence(r_+i,c_+2);
                double p[4]{p0,p1,p2,p3};
                bicubicInterpolator(p,delta_c,fc+i,dfdc+i);
            }

            // 求解 dfdr 和 f
            double evaluation_;
            double dfdr_;
            bicubicInterpolator(fc,delta_r,&evaluation_,&dfdr_);
            
            // 求解 dfdc
            double dfdc_;
            bicubicInterpolator(dfdc,delta_r,&dfdc_,NULL);


            *f = evaluation_;
            if(dfdc__!=NULL)
            *dfdc__ = dfdc_;
            if(dfdr__!=NULL)
            *dfdr__ = dfdr_;
        }


        // 插值函数
        void  bicubicInterpolator(const double * p, const double x,
         double * f, double * df) const
        {

            CHECK_LT(x,1);
            const double a = 0.5 * (-p[0] + 3.0 * p[1] - 3.0 * p[2] + p[3]);
            const double b = 0.5 * (2.0 * p[0] - 5.0 * p[1] + 4.0 * p[2] - p[3]);
            const double c = 0.5 * (-p[0] + p[2]);
            const double d = p[1];

            if (f != NULL) {
               f[0] = d + x * (c + x * (b + x * a));
            }

            // dfdx = 3ax^2 + 2bx + c
            if (df != NULL) {
                df[0] = c + x * (2.0 * b + 3.0 * a * x);
            }
        }
        private:
        float scaleFactor_;
        double gird_size_;
        const std::vector<Eigen::Vector2f> & laserInfo_;
        const ProbabilityGridMap & submap_;

        float scaleFactor_;
        double gird_size_;
    };
} // namespace LwlSLAM

