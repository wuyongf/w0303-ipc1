#pragma once

#include "al_common.h"
#include "data.h"

namespace yf
{
    namespace algorithm
    {
        class arm_path
        {
        public:
            arm_path(){}
            virtual ~arm_path(){}

            void RecordRefPath()
            {
                //todo:
                // 1. read ref_path from .txt file.
                // 2. record the LM1 position.
                // 3. record the transformation T2. T3(ref_path to base) = T1(LM1 to base)* T2;

            }

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
