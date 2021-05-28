#include <bits/stdc++.h>
// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int main()
{
    Mat img(300,300,CV_8UC1,Scalar(255));

    for(int i=100; i<200; i++)
        for(int j=100; j<200; j++)
            img.at<uchar>(i, j) = 0;
        
    imwrite("../data/obstacles/2D_Images/test.png",img); 
    
    Mat image = cv::imread("/home/rajat/melodic_ws/src/C-MINT/data/obstacles/2D_Images/test.png", 0);
    for(int i=0; i<image.rows; i++)
        for(int j=0; j<image.cols; j++)
        {
            if((int)image.at<uchar>(i, j) != 0 && (int)image.at<uchar>(i, j) != 255)
                std::cout<<(int)image.at<uchar>(i, j)<<" ";
        }
    return 0;
}