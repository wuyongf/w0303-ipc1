
#if 1

#include <iostream>
#include <string>


#include "../include/backup_manager_nwsys.h"


//bool CheckSystemStatus() {
//
//	//TODO:
//	//1. Check arm status
//	//2. Check mir status
//	//3. Check vision status
//
//	sys.status_arm = true;
//	sys.status_mir = true;
//	sys.status_vision = true;
//
//	if (sys.status_arm == false) {
//		std::cerr << "Arm Error!" << std::endl;
//	}
//
//	if (sys.status_mir == false) {
//		std::cerr << "MiR Error!" << std::endl;
//	}
//
//	if (sys.status_vision == false) {
//		std::cerr << "Vision Error!" << std::endl;
//	}
//
//	if (sys.status_arm == false || sys.status_mir == false || sys.status_vision == false) {
//		return 0;
//	}
//
//	sys.status_sys = true;
//	return 1;
//
//};

void Startup() {

	//TODO: startup Arm. MiR. TopModule. Vision.
	std::shared_ptr<ArmManager> Arm = std::make_shared<ArmManager>("tm5-900","192.168.2.29");
    std::shared_ptr<UgvManager> Ugv = std::make_shared<UgvManager>("mir100","192.168.2.xx");

    std::cout << "Arm model name: " << Arm->get_model_name() << std::endl;
    std::cout << "Arm ip address: " << Arm->get_ip_address() << std::endl;
    std::cout << "Ugv model name: " << Ugv->get_model_name() << std::endl;
    std::cout << "Arm ip address: " << Ugv->get_ip_address() << std::endl;

    //TODO: call the connection function, establish solid communication.

//	//TODO: startup status manager. check status
//
//	bool status_flag = CheckSystemStatus(); // 1: ok  0: error
//
//	if (status_flag == 0) {
//		std::cerr << "Sys Status Error! Quiting..." << std::endl;
//		return;
//	}
//
//	std::cout << "status flag is " << status_flag <<std::endl;
//	std::cout << "startup the sys... "<< std::endl;

}

int main() {
 
	do
	{
		std::cout << "Press key '1' to continue..." << std::endl;;
	} while (std::cin.get() != '1');

	Startup(); 
	
	return 0;
}

//void SystemStatusLog::system_status_log() {
//
//}

#endif

#pragma region EigenTesting
#if 0

#include <iostream>
#include "../Eigen3/Eigen/Core"
#include "../Eigen3/Eigen/Dense"

int print_eigen(Eigen::MatrixX3d m)
{
	// Eigen Matrices do have rule to print them with std::cout
	std::cout << m << std::endl;
	return 0;
}

int main()
{
	Eigen::Matrix3d test; //3 by 3 double precision matrix initialization

	// Let's make it a symmetric matrix
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			test(i, j) = (i + 1) * (1 + j);
	}

	// Print
	print_eigen(test);

	return 0;
}
#endif
#pragma endregion