// c_testing.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <thread>
#include <msclr/marshal_cppstd.h>
#using "d:/nw_sys/nw_sql_application_dll.dll"   //ÒýÓÃdll

using namespace nw_sql_application_dll;
using namespace std;

int check_flag;


void thread_function() {

    int i = 1, j, k, no_of_task, temp_job_id = 0, schedule_type;
    std::string temp[10];
    database_operation_class^ database_operation1 = gcnew database_operation_class();
    System::String^ temp_string;

    do
    {
        if (check_flag == 1)
        {
            temp_job_id = 0;
            cout << "Carry out new operation!" << endl;
            no_of_task = database_operation1->get_latest_job_task(); // get all tasks of 1st available schedule operation 
            if (no_of_task > 0)
            {
                database_operation1->update_device_current_status_and_log(1, 2); // 1 para: device id - 1 robot arm, 2nd para: new status - 1 idle, 2 working, 3 error, 4 breakdown
                schedule_type = database_operation1->get_schedule_type(database_operation1->task_detail[0]->schedule_id); // get the type of schedule operation
                database_operation1->update_schedule_status(database_operation1->task_detail[0]->schedule_id, 2); // status=2, in progress
                database_operation1->update_schedule_record(database_operation1->task_detail[0]->schedule_id, 2); // status=2, in progress
            }
            for (j = 0; j < no_of_task; j++)
            {
                temp_string = database_operation1->task_detail[j]->building;
                temp[0] = msclr::interop::marshal_as< std::string >(temp_string);
                temp_string = database_operation1->task_detail[j]->floor;
                temp[1] = msclr::interop::marshal_as< std::string >(temp_string);
                temp_string = database_operation1->task_detail[j]->room;
                temp[2] = msclr::interop::marshal_as< std::string >(temp_string);
                cout << "job:" << database_operation1->task_detail[j]->job_id << " building:" <<
                    temp[0].c_str() << " floor:" << temp[1].c_str()
                    << " room:" << temp[2].c_str() << " type:" <<
                    database_operation1->task_detail[j]->type << " x:" << database_operation1->task_detail[j]->x << " y:" <<
                    database_operation1->task_detail[j]->y << endl;
                if (temp_job_id != database_operation1->task_detail[j]->job_id)
                {
                    if (temp_job_id != 0)
                        database_operation1->update_job_record(temp_job_id, 3);
                    if (schedule_type == 1)
                    {
                        database_operation1->update_job_status(temp_job_id, 3);
                        database_operation1->update_job_status(database_operation1->task_detail[j]->job_id, 2); // 2nd para: new status - 2 in progress, 3 finish, 4 cancel
                    }
                    temp_job_id = database_operation1->task_detail[j]->job_id;
                    database_operation1->update_job_record(temp_job_id, 2);
                }
                if (schedule_type == 1)
                    database_operation1->update_task_status(database_operation1->task_detail[j]->task_id, 2); // 2nd para: new status - 2 in progress, 3 finish, 4 cancel
                database_operation1->update_task_record(database_operation1->task_detail[j]->task_id, 2);
                k = 0;
                do
                {   // sql waiting...
                    // robot operation
                    System::Threading::Thread::Sleep(5000);
                    cout << "Operation finish? (1=y,else=n)" << endl;
                    cin >> k;
                } while (k != 1);
                database_operation1->update_task_record(database_operation1->task_detail[j]->task_id, 3);
                if (schedule_type == 1)
                    database_operation1->update_task_status(database_operation1->task_detail[j]->task_id, 3); // 2nd para: new status - 2 in progress, 3 finish, 4 cancel

                // call robot operation
                k = 0;
            }

            if (no_of_task > 0)
            {
                database_operation1->update_job_record(temp_job_id, 3);
                database_operation1->update_job_status(temp_job_id, 3);
                check_flag = 2;
            }
            else
                check_flag = 0;
        }
        if (check_flag == 2)
        {
            // System::Threading::Thread::Sleep(5000);
            // cout << "Operation finish? (1=y,else=n)" << endl;
           // cin >> j;
           //  if (j == 1)
            {
                temp_job_id = 0;
                //for (j = 0; j < no_of_task; j++)
                //{
                //    if (temp_job_id != database_operation1->task_detail[j]->job_id)
                //    {
                //        temp_job_id = database_operation1->task_detail[j]->job_id;
                //        database_operation1->update_job_status(temp_job_id, 3); // 2nd para: new status - 2 in progress, 3 finish, 4 cancel
                //    }
                //    database_operation1->update_task_status(database_operation1->task_detail[j]->task_id, 3); // 2nd para: new status - 2 in progress, 3 finish, 4 cancel
                //}
                if (database_operation1->get_schedule_type(database_operation1->task_detail[0]->schedule_id) == 1) // schedule operation type=one time 
                    database_operation1->update_schedule_status(database_operation1->task_detail[0]->schedule_id, 3);
                database_operation1->update_schedule_record(database_operation1->task_detail[0]->schedule_id, 3);
                database_operation1->update_device_current_status_and_log(1, 1);
                check_flag = 0;
            }
            cout << "Finish schedule operation!" << endl;
        }
    } while (i == 1);
}

void thread_function2()
{
    database_operation_class^ database_operation = gcnew database_operation_class();
    int i = 1, operation_available;

    cout << "Start monitor new scheduled operation!" << endl;
    do
    {
        if (check_flag == 0)
            operation_available = database_operation->check_all_schedule_operation_available();
        else
            operation_available = 0;
        if (operation_available > 0)
        {
            cout << "New scheduled operation is ready!" << endl;
            check_flag = 1;
        }
        System::Threading::Thread::Sleep(2000);
    } while (i == 1);
}
int main()
{


    int no_of_task;
    //database_operation_class^ database_operation = gcnew database_operation_class();

    //no_of_task = database_operation->get_latest_job_task();
    //no_of_task = database_operation->task_detail[4]->job_id;
    check_flag = 0;
    thread t1(thread_function);
    thread t2(thread_function2);
    t1.join();
    t2.join();
    return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
