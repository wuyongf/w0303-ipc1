#if 0

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

int main(int argc, char** argv )
{
    std::string image_path = ("C:/Dev/w0303/Arm_Control_Module_v1.0/data/12w-lobby-238.jpg");
    Mat image = imread( image_path, 1 );

    if ( image.empty())
    {
        std::cout << "Could not read the image: " << image_path << std::endl;
        return -1;
    }

    imshow("Display Image", image);
    waitKey(0);
    return 0;
}
#endif



#if 1
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>

#include <iostream>


using namespace cv;

Mat img;

double x_start = 0, y_start = 0;
double x_end = 0, y_end = 0;
double delta_x;
double delta_y;
double theta;

void onMouse(int event, int x, int y, int flags, void* param)
{
    int y_for_mir = std::abs(y - img.rows);

    char text[100];
    Mat img2, img3;

    img2 = img.clone();

    if (event == EVENT_LBUTTONDOWN)
    {
        x_start = x;
        y_start = y_for_mir;

        Vec3b p = img2.at<Vec3b>(y,x);
        sprintf(text, "R=%d, G=%d, B=%d", p[2], p[1], p[0]);
    }
    else if (event == EVENT_RBUTTONDOWN)
    {
        cvtColor(img, img3, COLOR_BGR2HSV);
        Vec3b p = img3.at<Vec3b>(y,x);
        sprintf(text, "H=%d, S=%d, V=%d", p[0], p[1], p[2]);
    }
    else if (event == EVENT_LBUTTONUP)
    {
        x_end = x;
    y_end = y_for_mir;

    if (x_end != x_start || y_end != y_start)
    {
        delta_x = x_end - x_start;
        delta_y = y_end - y_start;

        theta =  std::atan2(delta_y,delta_x) * 180 / 3.1415926;

        std::cout << "theta: " << theta << std::endl;
    }
}
else
sprintf(text, "x=%d, y=%d", x, y_for_mir);

    putText(img2, text, Point(5,15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0));
    imshow("image", img2);


}

int main(int argc, char** argv)
{
    std::string image_path = ("C:/Dev/w0303/Arm_Control_Module_v1.0/data/12w-lobby-238.jpg");

    img = imread( image_path, 1 );

    if (img.empty())
        return -1;

    std::cout << "image column [x]: " << img.cols << "image row [y]: " << img.rows << std::endl;

    namedWindow("image");
    setMouseCallback("image", onMouse, 0);
    imshow("image", img);
    waitKey(0);

    return 0;
}

#endif