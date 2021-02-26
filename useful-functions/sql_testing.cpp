
#include "../include/sql.h"

int main()
{
    yf::sql::sql_server sql;

    std::cout << "ODBCConnectionStr: " << sql.getODBCConnectionStr() << std::endl;
    //"Driver={SQL Server};Server=192.168.0.8;Database=NW_mobile_robot_system;Uid=sa;Pwd=Willsonic2010"

    return 1;
}