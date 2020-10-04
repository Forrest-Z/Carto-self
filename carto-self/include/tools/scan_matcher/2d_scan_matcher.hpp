/*
 * @Author: Liu Weilong 
 * @Date: 2020-09-04 19:01:23 
 * @Last Modified by: Liu Weilong
 * @Last Modified time: 2020-09-04 19:16:50
 */

#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <math.h>

#include "glog/logging.h"
#include "tools/structure/mapping_structure.hpp"
#include "tools/structure/matching_structure.hpp"
#include "common.hpp"

#include "sensor_msgs/LaserScan.h"
#include "Eigen/Eigen"
#include "yaml-cpp/yaml.h"


namespace LwlSLAM
{

    class CorrelativeScanMatcher2d 
    {
        public:
        
        CorrelativeScanMatcher2d()=delete;

        CorrelativeScanMatcher2d(const std::string & configFile):configFile_(configFile){
            CHECK(initialFlag_ ==false);
        }

        LwlSLAM::MatchResult pipeline(const ProbabilityGridMap & submap, 
        const Eigen::Vector3f & posePrediction, const sensor_msgs::LaserScan & laserInfo);

        const MatchResult & getResult();

        private:

        void initial(const sensor_msgs::LaserScan & laserInfo);

        void dataPrepare();

        void generateSearchWindow(const sensor_msgs::LaserScan & laserInfo);

        void generateRotationDiscreteScans(const sensor_msgs::LaserScan & laserInfo,
        const Eigen::Vector3f & posePrediction);

        void generateTranslateCandidate();

        MatchResult scoreAllCandidates(const ProbabilityGridMap & submap,
        Eigen::Vector3f pose);

        MatchResult scoreAllCandidatesForOneRotation(const ProbabilityGridMap & submap,
        const std::vector<Eigen::Array2i> & rotationScan);

        void clearMidProduct();

        bool initialFlag_;
        bool usingEffectiveMaxRange_;
        const std::string configFile_;

        std::vector<float> tmpMidProductAngleOffset_;
        std::vector<Eigen::Array2i> tmpMidProductTranslationCandidates_;
        std::vector<std::vector<Eigen::Array2i>> tmpMidProductDiscreteScans_;

        SearchWindowParameter searchWindowParams_;
        MatchResult matchResult_;
    };
} // namespace name
