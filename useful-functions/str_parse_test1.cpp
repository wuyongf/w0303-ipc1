//
// Created by yongf on 2021/4/12.
//

#include <thread>

//glog
#include <glog/logging.h>

//Net
#include "../include/net.h"
#include "../include/net_w0303_server.h"
#include "../include/net_w0303_client.h"

// Database
#include "../include/sql.h"

// Status
#include "../include/nw_status.h"
// Arm
#include "../include/arm.h"
// UGV
#include "../include/ugv.h"
// IPC2
#include "../include/ipc2.h"

int main()
{
    std::deque<std::string> q_landmark_pos;

    yf::data::arm::Point3d landmark_pos;

    std::string original_str = "{1.112,2.3232,33234.444,4.1234,5.07,6.66}";

    auto index_left_curly_bracket = original_str.find("{");
    auto index_right_curly_bracket = original_str.find("}");


    std::cout << "index_left_curly_bracket: " << index_left_curly_bracket << std::endl;
    std::cout << "index_right_curly_bracket: " << index_right_curly_bracket << std::endl;

    auto index_comma = original_str.find(",");

    auto s = original_str.substr(1,index_right_curly_bracket-1 );

    std::cout << "s" << s << std::endl;

    std::string delimiter = ",";

    size_t pos = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        std::cout << token << std::endl;
        q_landmark_pos.push_back(token);
        s.erase(0, pos + delimiter.length());
    }
    // last token
    std::cout << s << std::endl;
    q_landmark_pos.push_back(s);

    landmark_pos.x = std::stof(q_landmark_pos[0]);
    landmark_pos.y = std::stof(q_landmark_pos[1]);
    landmark_pos.z = std::stof(q_landmark_pos[2]);
    landmark_pos.rx = std::stof(q_landmark_pos[3]);
    landmark_pos.ry = std::stof(q_landmark_pos[4]);
    landmark_pos.rz = std::stof(q_landmark_pos[5]);


    std::cout << s << std::endl;

}
