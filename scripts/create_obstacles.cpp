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
    for(int i=0; i<10;i++)
    {    
        Mat img(300,300,CV_8UC1,Scalar(255));
        int k=10;
        while(k--)
        {
            int x = rand() %300;
            int y = rand() %300;
            int length = 30 + rand() %(30);
            int breadth = 30 + rand() %(30);

            img = rectangle(img,x,y,length,breadth);
        }
        imwrite("../data/obstacles/2D_Images/"+to_string(i)+".png",img); 
    }
    return 0;
}