#include <iostream>
#include <sstream>
#include <iomanip>

#include "../include/al_common.h"

using namespace std;

union ulf
{
    unsigned long ul;
    float f;
};

union uuf
{
    uint32_t x;
    float f;
};

int main()
{

    auto z = unit32_to_float(17430,2607);

#if 0
    ulf u;
    string str = "408feb90";
    stringstream ss(str);
    ss >> hex >> u.ul;
    float f = u.f;
    cout << f << endl;
#endif
// 49869 8628 // 17430 2607 // 49969 24673
    uint16_t data[2]={17430,2607};

    uuf u;
    u.x = (((unsigned long)data[0] << 16) | data[1]);

    float result = u.f;

    std::cout << "x: " << result << std::endl;
}