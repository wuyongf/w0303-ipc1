// call_offset_dll.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>

#include <Windows.h>
#include <stdio.h>

#include "../include/data_arm.h"

using namespace std;

struct robot_path_data {
    int no_of_point;
    double z;
    double point[500][3];

    robot_path_data()
    {
        no_of_point = 0;
        z = 0;

        for (int i = 0; i < 500; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                point[i][j] = 0;
            }
        }
    };
};

double curve_get_offset_distance(const int& layer_no, const std::deque<yf::data::arm::Point3d>& ref_init_points)
{
    auto start_point = ref_init_points[0];
    auto end_point = ref_init_points[ref_init_points.size()-1];

    float   vec_max_magnitude = 0;
    int     farthest_point_no;

    for(int n = 1; n < ref_init_points.size()-1 ; n ++)
    {
        auto vec_1_x = start_point.x - ref_init_points[n].x;
        auto vec_1_y = start_point.y - ref_init_points[n].y;

        auto vec_2_x = end_point.x - ref_init_points[n].x;
        auto vec_2_y = end_point.y - ref_init_points[n].y;

        auto vec_3_x = (vec_1_x+vec_2_x)/2;
        auto vec_3_y = (vec_1_y+vec_2_y)/2;

        auto vec_3_magnitude = std::sqrtf(vec_3_x*vec_3_x + vec_3_y*vec_3_y);

        if(vec_max_magnitude < vec_3_magnitude)
        {
            vec_max_magnitude = vec_3_magnitude;
            farthest_point_no = n+1;
        }
    }

    return vec_max_magnitude/(layer_no+1);
}

typedef int(*getoffsetpath)(robot_path_data*,double,robot_path_data*[]);

typedef int (*get_point_cloud_from_depth_camera)(double [][3]);

int main()
{

    // Get the Point Clouds

    double input_pt[20000][3];

    char filename2[] = "rs-pointcloud.dll"; // in debug file
    wchar_t wtext2[100];
    mbstowcs(wtext2, filename2, strlen(filename2) + 1);
    LPWSTR ptr2 = wtext2;
    HINSTANCE hinstLib2 = LoadLibraryW(ptr2);

    //    HINSTANCE hinstLib2 = LoadLibrary(TEXT("../lib//rs-pointcloud.dll"));

    if (hinstLib2 == NULL)
    {
        return 1;
    }
    get_point_cloud_from_depth_camera get_point_cloud;
    get_point_cloud=(get_point_cloud_from_depth_camera)GetProcAddress(hinstLib2, "get_point_cloud_from_camera");
    int m;
    m=get_point_cloud(input_pt);


    // write the point cloud file
    ///\param file_name
    ///\param directory

    // format
    ofstream myfile ("c:/example.txt");
    if (myfile.is_open())
    {
        myfile << "# .PCD v.7 - Point Cloud Data file format\n";
        myfile << "VERSION .7\n";
        myfile << "FIELDS x y z\n";
        myfile << "SIZE 4 4 4\n";
        myfile << "TYPE F F F\n";
        myfile << "COUNT 1 1 1\n";
        myfile << "WIDTH " << m << "\n";
        myfile << "HEIGHT 1\n";
        myfile << "VIEWPOINT 0 0 0 1 0 0 0\n";
        myfile << "POINTS " << m << "\n";
        myfile << "DATA ascii\n";

        for(int count = 0; count < m; count ++)
        {
            for (int index = 0 ; index < 3; index++)
            {
                myfile << input_pt[count][index] << " " ;
            }
            myfile << std::endl ;
        }
        myfile.close();
    }
    else cout << "Unable to open file";




#if 0    /// rear
    //    pt[0][0] = -717.7292; // x0
//    pt[0][1] = -368.8069; // y0
//    pt[0][2] = 0;
//    pt[1][0] = -515.1719; // x1
//    pt[1][1] = -244.5387; // y1
//    pt[1][2] = 0;
//    pt[2][0] = -592.2229; // x2
//    pt[2][1] = -96.76231; // y2
//    pt[2][2] = 0;
//    pt[3][0] = -571.4864; // x3
//    pt[3][1] = 75.99839; // y3
//    pt[3][2] = 0;
//    pt[4][0] = -772.9131; // x4
//    pt[4][1] = 144.4268; // y4
//    pt[4][2] = 0;
#endif

#if 0    /// left
    pt[0][0] = -357.5795; // x0
    pt[0][1] = 759.9092; // y0
    pt[0][2] = 0;
    pt[1][0] = -234.9845; // x1
    pt[1][1] = 553.1698; // y1
    pt[1][2] = 0;
    pt[2][0] = -67.39543; // x2
    pt[2][1] = 636.0949; // y2
    pt[2][2] = 0;
    pt[3][0] = 107.8367; // x3
    pt[3][1] = 601.476; // y3
    pt[3][2] = 0;
    pt[4][0] = 132.8193; // x4
    pt[4][1] = 797.4377; // y4
    pt[4][2] = 0;
#endif

    ///yf: Initialization
    yf::data::arm::Point3d p1,p2,p3,p4,p5;

    p1.x = -717.7292; p1.y = -368.8069; p1.z = 0; p1.rx = 0; p1.ry = 0; p1.rz = 0;
    p2.x = -515.1719; p2.y = -244.5387;
    p3.x = -592.2229; p3.y = -96.76231;
    p4.x = -571.4864; p4.y = 75.99839;
    p5.x = -772.9131; p5.y = 144.4268;

    std::deque<yf::data::arm::Point3d> ref_init_points;

    ref_init_points.push_back(p1);
    ref_init_points.push_back(p2);
    ref_init_points.push_back(p3);
    ref_init_points.push_back(p4);
    ref_init_points.push_back(p5);


    ///yf: layer decide offset
    // find height

    int layer = 4;

    double  offset = curve_get_offset_distance(layer, ref_init_points);

    /// ref_init_points ---> double array
    double ptt[500][3];

    // array initialization
    for (auto & i : ptt)
    {
        for(double & j : i)
        {
            j = 0;
        }
    }

    // ref_init_points ---> array
    for(int n = 0; n < ref_init_points.size(); n++)
    {
        ptt[n][0] = ref_init_points[n].x;
        ptt[n][1] = ref_init_points[n].y;
        ptt[n][2] = 0;
    }

    /// Offset Method input
    int i,j,no_of_offset_path;

    robot_path_data init_path,*offset_path[50];

    getoffsetpath get_offset_path;

    HINSTANCE hinstLib = LoadLibrary(TEXT("c:\\polylineoffset.dll"));

    if (hinstLib == NULL)
    {
        return 1;
    }

    /// Offset Method input: point size
    init_path.no_of_point = ref_init_points.size();

    /// Offset Method output
    for (i = 0; i < init_path.no_of_point; i++)
    {   for (j=0;j<3;j++)
            init_path.point[i][j] = ptt[i][j];
    }
    get_offset_path=(getoffsetpath)GetProcAddress(hinstLib, "find_offset_paths");
    no_of_offset_path=get_offset_path(&init_path,offset,offset_path);


    /// yf
    std::deque<yf::data::arm::Point3d> ref_path;

    for(int n = 0 ; n < no_of_offset_path; n++)
    {
        std::deque<yf::data::arm::Point3d> ref_path_each_layer;

        /// For Each Layer, Find All Points
        auto each_layer_path = offset_path[n];

        // 1. find each layer's point number.
        int each_layer_points_number = each_layer_path->no_of_point;

        // assign to std container
        for (int m = 0; m < each_layer_points_number; m++)
        {
            yf::data::arm::Point3d point;

            point.x = each_layer_path->point[m][0];
            point.y = each_layer_path->point[m][1];

            //
            point.z  = ref_init_points[0].z;
            point.rx = ref_init_points[0].rx;
            point.ry = ref_init_points[0].ry;
            point.rz = ref_init_points[0].rz;

            ref_path_each_layer.push_back(point);
        }

        // rearrange path. check even
        if(n % 2 != 0)
        {
            std::reverse(ref_path_each_layer.begin(), ref_path_each_layer.end());
        }

        // insert to ref_paths
        ref_path.insert(ref_path.end(), ref_path_each_layer.begin(), ref_path_each_layer.end());
    }
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
