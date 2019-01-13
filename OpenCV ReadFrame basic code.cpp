#include<iostream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;

Mat function(Mat);

int main()
{
    VideoCapture cam(0);
    if(cam.isOpened() == false)
    {
        cout << "Camera not found" << endl;
        return 0;
    }
    Mat orig;
   // namedWindow("Track");
   // createTrackbar("Thresh","Track",&iter,255);
    while(1)
    {
        if(!cam.read(orig))
        {
            cout << "No fram read" << endl;
            break;
        }



        imshow("Original",orig);
        if(waitKey(10) == 27)
                break;
    }
}

Mat function(Mat src)
{
    Mat image = src.clone();
    Mat dst;
    return dst;
}
