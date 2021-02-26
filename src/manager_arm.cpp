#include <iostream>
#include <string>

#include "../include/manager_arm.h"

int main()
{
    std::shared_ptr<yf::manager::arm_manager> tm5 = std::make_shared<yf::manager::arm_manager>();

    std::cout << "model name: " << tm5->GetModelName() << std::endl;

    yf::




}