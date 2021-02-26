#pragma once

#include "data_common.h"
#include "data_arm.h"

namespace yf
{
    namespace data
    {
        namespace models
        {
            //Item
            enum class ModelName
            {
                Null        = 0,
                Handrail    = 1,
                Chair       = 2,
                Floor       = 3,  // no need.
                Wall        = 4,
                Handle      = 5,
                Lift_Button = 6,
                Desk        = 7,
                Skirt       = 8
            };

            struct Surface
            {
                int surface_no;            // two surfaces the arm needs to clean... push back to vector..

                // basic info about dimension
                float height;
                float width;
                float length;

                yf::data::arm::Point3d ref_landmark_position;
                yf::data::arm::Point3d ref_init_position;            // record to tm robot base.
                std::vector<yf::data::arm::Point3d> ref_path;

                yf::data::arm::Point3d real_landmark_position;
                yf::data::arm::Point3d real_init_position;           // record to tm robot base.
                std::vector<yf::data::arm::Point3d> real_path;
            };

            struct RefModel
            {
                //basic ref model info
                int id = 0;
                ModelName name;

                std::vector<Surface> surfaces;

                std::vector<yf::data::arm::Point3d> clean_path;
                std::vector<yf::data::arm::Point3d> disinfection_path;
            };
        }
    }

}


