#include <iostream>
#include <string>
#include <sstream>

// todo: checksum. compute the value between '$' and ','
std::string ChecksumStr(const std::string& str)
{
    uint16_t checksum = 0;

    unsigned int index = 1; // Skip $ character at beginning
    const unsigned int length = str.length();
    for (; (index < length); ++index)
    {
        const char16_t c = str[index];
        if (c == '*')
            break;
        checksum = checksum ^ c;
    }

    std::stringstream stream;
    stream <<  std::hex<<checksum;

    std::string result( stream.str() );

    return result;
}

//input:
//output:

std::string ConstructStr(const std::string& ref_str)
{
    std::string data = "1,\r\n" + ref_str ;
    std::cout << "data: " << data << std::endl;

    std::string size = std::to_string(data.size());
    std::cout << "size: " << size << std::endl;

    std::string checksum_str = "$TMSCT," + size + "," + data + ",*";
    return checksum_str;
}

std::string ConstrucLnStr(const std::string& str)
{
    std::string checksum_str =  ConstructStr(str);

    uint16_t checksum = 0;

    unsigned int index = 1; // Skip $ character at beginning
    const unsigned int length = checksum_str.length();
    for (; (index < length); ++index)
    {
        const char16_t c = checksum_str[index];
        if (c == '*')
            break;
        checksum = checksum ^ c;
    }

    std::stringstream stream;
    stream <<  std::hex << checksum;

    std::string result( stream.str() );

    std::string final_str = checksum_str + result;

    return final_str;
}

int main() {

    std::string final_str = ConstrucLnStr("float[] targetP1= {-20,-4,72,14,86,-1}\r\n"
                                          "PTP(\"JPP\",targetP1,10,200,0,false)");

    std::cout << "final_str: " <<   std::endl;
    std::cout << final_str << std::endl;

}
