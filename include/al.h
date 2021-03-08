#pragma once

#include "al_common.h"
#include "data.h"

namespace yf
{
    namespace algorithm
    {
        class degree_related
        {
        public:
            degree_related(){}
            virtual ~degree_related(){}

            float d2r(const float& degree);
            float r2d(const float& radian);

            Eigen::Matrix3f ryp2RMat(const float& roll, const float& pitch, const float& yaw);
            Eigen::Matrix4f points2TMat(std::vector<float>& point);

            std::vector<float> R2rpy(Eigen::Matrix3f& RMat);
        };

        class arm_path
        {
        public:
            arm_path(){}
            virtual ~arm_path(){}

            void RecordRefPath();

            void ExportRealPathByLM()
            {
                //todo: Find T6 = T4*inv(T1)*T3
                // 1. retrieve T2
                // 2. Find LM2 position. T4
                // 3. calculate the real_path.

            }


        };
    }
}

void yf::algorithm::arm_path::RecordRefPath()
{
    //todo:
    // 1. read ref_path from .txt file.
    // 2. record the LM1 position.
    // 3. record the transformation T2. T3(ref_path to base) = T1(LM1 to base)* T2;

}