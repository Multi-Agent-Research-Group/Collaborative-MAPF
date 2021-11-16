 
#include <bits/stdc++.h>
// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int isValid(int i , int j , Mat img)
{
    if ( i < img.rows && j < img.cols && i > 0 && j > 0)
        return 1;
    return 0;
}

Mat circle(Mat a, int i, int j, int r)
{
    int p,q;
    i = i-r;
    j = j-r;
    for(p=i-r;p<i+r;p++)
        for(q=j-sqrt(pow(r,2)-pow(p-i,2));q<j+sqrt(pow(r,2)-pow(p-i,2));q++)
        {
            if(isValid(q,p,a))
                a.at<uchar>(p,q)=0;
        }

    return a;
}

Mat square(Mat a , int j , int k , int r )
{
    int m,n;
    for(m = j - r; m <= j + r; m++)
    {
        for(n = k - r; n <= k + r; n++)
        {
            if(isValid(m, n, a))
            {
                a.at<uchar>(m,n) = 0;
            }
        }
    }
    return a;
}

Mat rectangle(Mat a , int j , int k , int length, int breadth )
{
    int m,n;
    for(m = j - length/2; m <= j + length/2; m++)
    {
        for(n = k - breadth/2; n <= k + breadth/2; n++)
        {
            if(isValid(m, n, a))
            {
                a.at<uchar>(m,n) = 0;
            }
        }
    }
    return a;
}

Mat triangle(Mat a , int j , int k , int r )
{
    int m,n;
    for(m = j - r; m <= j + r; m++)
    {
        for(n = m; n <= k + r; n++)
        {
            if(isValid(m, n, a))
            {
                a.at<uchar>(m,n) = 0;
            }
        }
    }
    return a;
}

int main()
{
    srand(time(0));
    // vector<vector<int>> v {
    // {0,0,0,0,1,0,0,0,0},
    // {0,0,0,0,1,0,0,0,0},
    // {0,0,0,0,0,0,0,0,0},
    // {0,0,0,0,1,0,0,0,0},
    // {1,1,0,1,1,1,0,1,1},
    // {0,0,0,0,1,0,0,0,0},
    // {0,0,0,0,0,0,0,0,0},
    // {0,0,0,0,1,0,0,0,0},
    // {0,0,0,0,1,0,0,0,0}
    // };
    // vector<vector<int>> v {
    // {0,0,0,0,0,0,0,0,0},
    // {0,0,0,1,1,1,0,0,0},
    // {0,0,0,0,1,0,0,0,0},
    // {0,1,0,0,1,0,0,1,0},
    // {0,1,1,1,1,1,1,1,0},
    // {0,1,0,0,1,0,0,1,0},
    // {0,0,0,0,1,0,0,0,0},
    // {0,0,0,1,1,1,0,0,0},
    // {0,0,0,0,0,0,0,0,0}
    // };

    vector<vector<int>> v {
    {0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0}
    };
    Mat img(1000,1000,CV_8UC1,Scalar(255));
    int length = 100, breadth = 100;
    for(int i=0; i<v.size();i++)
        for(int j=0; j<v.size(); j++)
            if(v[i][j])
                img = rectangle(img,(i+1)*100,(j+1)*100,length,breadth); 

    std::cin.get();
    for(int i=0; i<=v.size()+1;i++)
    {
        img = rectangle(img,i*100,0,length,breadth); 
        img = rectangle(img,i*100,1000,length,breadth); 
        img = rectangle(img,0,i*100,length,breadth); 
        img = rectangle(img,1000,i*100,length,breadth); 
    }
    cv::namedWindow("Agents",cv::WINDOW_NORMAL);
    cv::imshow("Agents", img);
    cv::waitKey(1);
    imwrite("../data/obstacles/check_empty.png",img);
    return 0;
}