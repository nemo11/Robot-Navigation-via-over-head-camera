#include<iostream>
#include<algorithm>
#include <cstdlib>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>       //displays images and gui
#include<opencv2/imgproc/imgproc.hpp>       //does main image processing
#include<cmath>
#include <QCoreApplication>
#include <QtSerialPort/QSerialPort>
#include <QDebug>
#include <QTime>
#include <QString>
#define EAST	0
#define NORTH	1
#define WEST	2
#define SOUTH	3
#define gridX 480
#define gridY 640
#define F	0
#define R	1
#define B	2
#define L	3

#define pi 3.14159265
using namespace cv;
using namespace std;
QSerialPort* serial;
Mat image,drawing_B;
int  x_ref, y_ref, r_x, r_y, r, g, b, v, xF, yF, xB, yB, x_t, y_t, bcx, bcy;
Mat thresh_TC, thresh_R, gray, cameraFeed, HSV, thresh_BF, thresh_BB;
int X[50], Y[50];
int no_path = 0;
int q, w ;
int dir, v_dir;
int dist; //stores the value of the current cordinate in the weighted matrix
int c = 0; //cer for output[]
int output[480 * 640], xo[480 * 640], yo[480 * 640]; //the array that the function returns
int temp_x, temp_y;
int c_x, c_y;
int wgrid[480][640];
int grid[480][640];
Mat gray, thresh;
int x, y, v, r, g, b, b_x, b_y, r_n;
Mat drawing;
Mat thresh_TC, thresh_R, cameraFeed, HSV, thresh_B;
int X[50], Y[50];
VideoCapture capture;
//Function prototypes
void find_path();
void var_init();
void create_weighted();
void show_weighted();
void execute_path();
void sort();

float x, y;
int t, l, c;
int dist1[50], dist2[50];
int n,s;
int D_X[50], D_Y[50];
double cc_angle(Point vtx,Point p1,Point p2);
double abs_distance(Point p1, Point p2);
void array_init(Mat image, int x[480][640])
{
    memset(x, 1, sizeof(x));
    for (int i = 0; i < 480; i++)
    {
        for (int j = 0; j < 640; j++)
        {
            int b = (int)image.at<Vec3b>(j, i)[0];
            int g = (int)image.at<Vec3b>(j, i)[1];
            int r = (int)image.at<Vec3b>(j, i)[2];
            /*if (r > 180 && g > 180 && r < 245 && g < 245 && b < 20)
            x[i][j] = 0;
            else*/
            if (r < 20 && g == 190 && b == 190)
            {
                for (int k = 0; k < 12; k++)
                {
                    for (int p = 0; p < 12; p++)
                    {
                        x[i + p][j + k] = 1000;
                        x[i - p][j - k] = 1000;
                        x[i + p][j - k] = 1000;
                        x[i - p][j + k] = 1000;

                    }
                }


            }
            /*else
            {
                //if (x[i][j] != 1000)
                x[i][j] = 1;
            }*/
        }
    }
}
void imageprocess()
{

    cvtColor(image, HSV, COLOR_BGR2HSV);
    inRange(HSV, Scalar(28, 0, 252), Scalar(180, 91, 255), thresh_R);
    //inRange(HSV, Scalar(6, 69, 67), Scalar(20, 255, 255), thresh_TC);
    //("thrsh_TC", thresh_TC);
    imshow("thrsh_R", thresh_R);
    cout<<"A"<<endl;
    vector< vector<Point> > resources;
    vector< vector<Point> > towncenter;
    vector<Vec4i> hierarchy;
    vector<Point> approx;
    //findContours(thresh_TC, towncenter, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    //Mat drawing_TC = Mat::zeros(thresh_TC.size(), CV_8UC3);
    findContours(thresh_R, resources, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    Mat drawing_R = Mat::zeros(thresh_R.size(), CV_8UC3);
    n=resources.size();
    cout<<"RESOU"<< n <<endl;
    /*for (int i = 0; i< towncenter.size(); i++)
    {
        drawContours(drawing_TC, towncenter, i, Scalar(0, 255, 0), 2, 8, hierarchy, 0, Point());
        Moments moment = moments((cv::Mat)towncenter[i]);
        double area = moment.m00;
        x_t = moment.m10 / area;
        y_t = moment.m01 / area;
    }*/
//int ctr=0;
    for (int i = 0; i < resources.size(); i++)
    {
        approxPolyDP(resources[i], approx, arcLength(resources[i], true)*0.04, true);

        Moments moment = moments((cv::Mat)resources[i]);
        double area = moment.m00;

      if (area>200)
        {
        drawContours(drawing_R, resources, i, Scalar(0, 255, 0), 2, 8, hierarchy, 0, Point());
        x = moment.m10 / area;
        y = moment.m01 / area;
        X[i] = x;
        Y[i] = y;
        cout<< X[i]<< "   "<<Y[i]<<endl;
  //      ctr++;
        cout<<"area"<< area<<endl;
        }

    }
    //n=ctr++;
    //imshow("drawing_TC", drawing_TC);
    imshow("drawing_RT", drawing_R);
}
void botprocess(int pos)
{
    cvtColor(image, HSV, COLOR_BGR2HSV);
   // cout<<"ri"<<endl;
    inRange(HSV, Scalar(0, 27, 232), Scalar(18, 255, 255), thresh_BF);
   // cout<<"R_"<<endl;
    inRange(HSV, Scalar(58, 25, 192), Scalar(82, 255, 255), thresh_BB);
   // cout<<"r"<<endl;
    erode(thresh_BF, thresh_BF, getStructuringElement(MORPH_RECT, Size(3, 3)));
    //erode(thresh_BB, thresh_BB, getStructuringElement(MORPH_RECT, Size(3, 3)));
    imshow("thresh_BF", thresh_BF);
    imshow("thresh_BB", thresh_BB);


    vector< vector<Point> > botF;
    vector< vector<Point> > botB;
    vector<Vec4i> hierarchy;
    vector<Point> approx;
   // cout<<"ii"<<endl;
    findContours(thresh_BF, botF, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(thresh_BB, botB, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
//    cout<<"io"<<endl;
    drawing_B = Mat::zeros(image.size(), CV_8UC3);
    //Mat temp;
    //bitwise_or(thresh_BB,thresh_BF,temp);
    //imshow("Thresh",temp);
    if(pos!=-1)
    for (int i = pos; i< pos+1; i++)
    {

        //drawContours(drawing_B, botF, i, Scalar(0, 0, 255), 2, 8, hierarchy);
        //drawContours(drawing_B, botB, i, Scalar(0, 255, 0), 2, 8, hierarchy);
       // cout<<"i"<<endl;
        Moments momentF = moments((cv::Mat)botF[0]);
        Moments momentB = moments((cv::Mat)botB[0]);
        double areaF = momentF.m00;
        double areaB = momentB.m00;
        //if (areaF>100 && areaB>100)
        xF = momentF.m10 / areaF;
        yF = momentF.m01 / areaF;
        xB = momentB.m10 / areaB;
        yB = momentB.m01 / areaB;
        bcx = (xF + xB) / 2;
        bcy = (yF + yB) / 2;
       // c = atan2((yF - yB) / (xF - xB)) * 180 / pi;
       // l = 180 - atan(fabsf(D_Y[i] - bcy) / fabsf(D_X[i] - bcx)) * 180 / pi;
       // cout<<"Calc "<<Point(D_X[i],D_Y[i])<<endl;
        c= cc_angle(Point(xB,yB) ,Point (xB+100,yB),Point (xF,yF));
        l= cc_angle(Point (bcx,bcy),Point (bcx+100,bcy),Point (D_X[i],D_Y[i]));
        t = l - c;
        t = (atan2(sin(t),cos(t)))*180/pi;
         if (t>=0)
            s=0;
        else s=1;
      //  cout<< -t << "  qwe  "<<endl;
        line(drawing_B, Point(xF, yF), Point(xB, yB), Scalar(0, 0, 255), 1);
        line(drawing_B, Point(bcx, bcy), Point(D_X[i], D_Y[i]), Scalar(0, 0, 255), 1);
     //   cout << t << endl;

    }
    //imshow("drawing_B", drawing_B);
}
void sort()
{
  int i,j,temp;
  x_t= (image.cols)/2;
  y_t= (image.rows)/2;
  cout<< x_t << "  "<< y_t<< endl;

  for (i = 0; i < n; i++)
    {
      cout<< "helo"<< X[i]<<"   "<<Y[i] <<endl;
        dist1[i] = (X[i] - x_t)*(X[i] - x_t) + (Y[i] - y_t)*(Y[i] - y_t);
        dist2[i] = (X[i] - bcx)*(X[i] - bcx) + (Y[i] - bcy)*(Y[i] - bcy);

        cout<< dist1[i]<< "    "<<dist2[i]<<endl;
                if (dist1[i] ==0 || dist1[i] == (bcx - x_t)*(bcx - x_t) + (bcy - y_t)*(bcy - y_t))
                      dist1[i]=1000000;
                if (dist2[i] ==0)
                      dist2[i]=1000000;

    }
int min=dist2[0];
for (i = 0; i < n; i++)
        {

            if (dist2[i] <= min )
            {	min = dist2[i];

                        }
        }
for (i = 0; i < n; i++)
                {
                    if (min == dist2[i])
                    {
                        D_X[0]=X[i];
                        D_Y[0]=Y[i];
                        dist1[i] = 1000000;
                    }
                }

for(i=1;i<n;i++)
{
  for(j=0;j<(n-i);j++)
  {
    if(dist1[j]>dist1[j+1])
    {
      temp=dist1[j];
      dist1[j]=dist1[j+1];
      dist1[j+1]=temp;
      temp=X[j];
      X[j]=X[j+1];
      X[j+1]=temp;
      temp=Y[j];
      Y[j]=Y[j+1];
      Y[j+1]=temp;
    }

  }

}
for(i=2;i<2*n+1;i+=2)
  {
  D_X[i]=X[(i-2)/2];
  D_Y[i]=Y[(i-2)/2];
  D_X[i-1]=x_t;
  D_Y[i-1]=y_t;
  }
for(i=0;i<2*n;i++)
    cout<< D_X[i]<< "   "<<D_Y[i]<<endl;

}
double abs_distance(Point p1, Point p2)
{
    double dist;
    dist=sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
    return dist;
}
double cc_angle(Point vtx,Point p1,Point p2)
{
// 1st point is the vertex , then 1st point and 2nd point,angle is obtained in radians
    double modv1 = abs_distance(vtx,p1);		// Mod v1
    double modv2 = abs_distance(vtx,p2);		// Mod v2
    double dot = (p1.x-vtx.x)*(p2.x-vtx.x) + (p1.y-vtx.y)*(p2.y-vtx.y);
    double costheta = dot/(modv1*modv2);
    double sign =0;
    double term = (p1.x-vtx.x)*(p2.y-vtx.y)-(p1.y-vtx.y)*(p2.x-vtx.x);
    if(term <= 0)
        sign = 1.0;
    else
        sign = -1.0;
    double theta = 0;
    if(costheta >=1.0)
        theta = 0;
    else if(costheta <=-1.0)
        theta = acos(-1.0);
    else
        theta = acos(costheta);
    return theta*sign;

}

int main(int argc, char *argv[])
{

    capture.open(1);
    capture.read(image);
    cout<< image.cols <<"     "<< image.rows<<endl;
    imshow("qqq",image);
    imageprocess();
    array_init(image, grid);
    botprocess(-1);
    sort();
    serial = new QSerialPort();                                     //serialcommunication part
    serial->setPortName("COM14");
    serial->open(QIODevice::ReadWrite);
    serial->setBaudRate(QSerialPort::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    for(int i=0;i< 2*n;i++)
    {
        memset(output, -1, sizeof(output));
                dir = NORTH;
                int c, i, j;

                q = X[k];
                w = Y[k];

                if (grid[q][w] > 990)
                {
                    no_path = 1;
                    cout << "Not possible";

                }
                find_path();
       // while(1)
    //{
    capture.read(image);
    botprocess(i);
    execute_path();
    QString send=QString::fromUtf8(output);
    //cout<<send.data()<<endl;
    serial->write(send.toUtf8());
    if (serial->bytesAvailable()>0||serial->waitForReadyRead(10))
            {
                QByteArray ba;
                ba=serial->readAll();
                qDebug()<<ba;
            }
    imshow("drawingB",drawing_B);
   // cout<<"Dist "<<abs_distance(Point(bcx,bcy),Point(D_X[i],D_Y[i]))<<endl;
    if (abs_distance(Point(bcx,bcy),Point(D_X[i],D_Y[i]))<10)
        break;
   // cout<<bcx<<" "<<bcy<<" +  "<<D_X[i]<<" "<<D_Y[i]<<endl;
    //waitKey(2);
      //  }
        cout<<"sarila"<<endl;
        dir = v_dir;
                    cout << "\n" << dir;
                    cout << "\n\n";
                    memset(output, -1, sizeof(output));
    }
    waitKey(0);
    return 0;
}
void var_init()
{
    //c_x = 0;
    //c_y = 0;
    temp_x = c_x;
    temp_y = c_y;
    dist = wgrid[c_x][c_y];
    c = 0;
    v_dir = dir;
}

void show_weighted()
{
    int i, j;
    for (i = 0; i<gridX; i++)
    {
        for (j = 0; j<gridY; j++)
        {
            cout << "\t" << wgrid[i][j];
        }
        cout << "\n";
    }
}

void create_weighted()
{
    int i, j;
    for (i = 0; i<gridX; i++)
    {
        for (j = 0; j<gridY; j++)
        {
            if (grid[i][j] < 1000)
            {
                wgrid[i][j] = a(q - i) + a(w - j);
            }

            else
            {
                wgrid[i][j] = grid[i][j];
            }
        }
    }
}

void execute_path()
{
    int i;
    for (i = 0; i<gridX*gridY; i++)
    {
        if (output[i] != 0)
        {

            switch (output[i])
            {

            //case 0:
            //cout << "F\t";
            //break;
            case 1:
                cout << "\nR " << xo[i] << " " << yo[i];
                    break;
            case 2:
                cout << "\nB " << xo[i] << " " << yo[i];
                    break;
            case 3:
                cout << "\nL " << xo[i] << " " << yo[i];
                    break;
            case 4:
            cout << "D\t";
            break;
            }
        }
    }
}

void find_path()
{

    create_weighted();
    var_init();
    //show_weighted();
    while (dist != 0)
    {
        if (v_dir == SOUTH)
        {
            if (wgrid[temp_x + 1][temp_y] < dist)
            {
                v_dir = SOUTH;
                output[c] = F;
                xo[c] = temp_x;
                yo[c] = temp_y;
                //line(frame, Point(temp_x, temp_y), Point(temp_x + 1, temp_y), Scalar(0, 255, 0), 2);
                dist = wgrid[temp_x + 1][temp_y];
                temp_x += 1;
                temp_y += 0;
            }

            else if (wgrid[temp_x][temp_y + 1] < dist)
            {
                v_dir = EAST;
                output[c] = L;
                xo[c] = temp_x;
                yo[c] = temp_y;
                //line(frame, Point(temp_x, temp_y), Point(temp_x, temp_y + 1), Scalar(0, 255, 0), 2);

                dist = wgrid[temp_x][temp_y + 1];
                temp_x += 0;
                temp_y += 1;
            }

            else if (wgrid[temp_x][temp_y - 1] < dist)
            {
                v_dir = WEST;
                //line(frame, Point(temp_x, temp_y), Point(temp_x, temp_y - 1), Scalar(0, 255, 0), 2);
                output[c] = R;
                xo[c] = temp_x;
                yo[c] = temp_y;
                dist = wgrid[temp_x][temp_y - 1];
                temp_x -= 0;
                temp_y -= 1;
            }
            else if (wgrid[temp_x - 1][temp_y] < dist)
            {
                v_dir = NORTH;
                //line(frame, Point(temp_x, temp_y), Point(temp_x - 1, temp_y), Scalar(0, 255, 0), 2);
                output[c] = B;
                xo[c] = temp_x;
                yo[c] = temp_y;
                dist = wgrid[temp_x - 1][temp_y];
                temp_x -= 1;
                temp_y -= 0;
            }
            else
            {
                wgrid[temp_x][temp_y] += 2;
                var_init();
                continue;
            }
            c++;
        }


        else if (v_dir == EAST)
        {
            if (wgrid[temp_x][temp_y + 1] < dist)
            {
                v_dir = EAST;
                //line(frame, Point(temp_x, temp_y), Point(temp_x, temp_y + 1), Scalar(0, 255, 0), 2);
                output[c] = F;
                xo[c] = temp_x;
                yo[c] = temp_y;
                dist = wgrid[temp_x][temp_y + 1];
                temp_x += 0;
                temp_y += 1;
            }

            else if (wgrid[temp_x + 1][temp_y] < dist)
            {
                v_dir = SOUTH;
                //line(frame, Point(temp_x, temp_y), Point(temp_x + 1, temp_y), Scalar(0, 255, 0), 2);
                output[c] = R;
                xo[c] = temp_x;
                yo[c] = temp_y;
                dist = wgrid[temp_x + 1][temp_y];
                temp_x += 1;
                temp_y += 0;
            }

            else if (wgrid[temp_x - 1][temp_y] < dist)
            {
                v_dir = NORTH;
                //line(frame, Point(temp_x, temp_y), Point(temp_x - 1, temp_y), Scalar(0, 255, 0), 2);
                output[c] = L;
                xo[c] = temp_x;
                yo[c] = temp_y;
                dist = wgrid[temp_x - 1][temp_y];
                temp_x -= 1;
                temp_y -= 0;
            }
            else if (wgrid[temp_x][temp_y - 1] < dist)
            {
                v_dir = WEST;
                output[c] = B;
                xo[c] = temp_x;
                yo[c] = temp_y;
                //line(frame, Point(temp_x, temp_y), Point(temp_x, temp_y - 1), Scalar(0, 255, 0), 2);

                dist = wgrid[temp_x][temp_y - 1];
                temp_x -= 0;
                temp_y -= 1;
            }
            else
            {
                wgrid[temp_x][temp_y] += 2;
                var_init();
                continue;
            }
            c++;
        }

        else if (v_dir == WEST)
        {

            if (wgrid[temp_x][temp_y - 1] < dist)
            {
                v_dir = WEST;
                output[c] = F;
                xo[c] = temp_x;
                yo[c] = temp_y;
                //line(frame, Point(temp_x, temp_y), Point(temp_x, temp_y - 1), Scalar(0, 255, 0), 2);

                dist = wgrid[temp_x][temp_y - 1];
                temp_x -= 0;
                temp_y -= 1;
            }

            else if (wgrid[temp_x + 1][temp_y] < dist)
            {
                v_dir = SOUTH;
                //line(frame, Point(temp_x, temp_y), Point(temp_x + 1, temp_y), Scalar(0, 255, 0), 2);
                output[c] = L;
                xo[c] = temp_x;
                yo[c] = temp_y;
                dist = wgrid[temp_x + 1][temp_y];
                temp_x += 1;
                temp_y += 0;
            }
            else if (wgrid[temp_x - 1][temp_y] < dist)
            {
                v_dir = NORTH;
                //line(frame, Point(temp_x, temp_y), Point(temp_x - 1, temp_y), Scalar(0, 255, 0), 2);
                output[c] = R;
                xo[c] = temp_x;
                yo[c] = temp_y;
                dist = wgrid[temp_x - 1][temp_y];
                temp_x -= 1;
                temp_y -= 0;
            }
            else if (wgrid[temp_x][temp_y + 1] < dist)
            {
                v_dir = EAST;
                //line(frame, Point(temp_x, temp_y), Point(temp_x, temp_y + 1), Scalar(0, 255, 0), 2);
                output[c] = B;
                xo[c] = temp_x;
                yo[c] = temp_y;
                dist = wgrid[temp_x][temp_y + 1];
                temp_x += 0;
                temp_y += 1;
            }

            else
            {
                wgrid[temp_x][temp_y] += 2;
                var_init();
                continue;
            }
            c++;
        }

        else if (v_dir == NORTH)
        {
            if (wgrid[temp_x - 1][temp_y] < dist)
            {
                v_dir = NORTH;
                //line(frame, Point(temp_x, temp_y), Point(temp_x - 1, temp_y), Scalar(0, 255, 0), 2);
                output[c] = F;
                xo[c] = temp_x;
                yo[c] = temp_y;
                dist = wgrid[temp_x - 1][temp_y];
                temp_x -= 1;
                temp_y -= 0;
            }

            else if (wgrid[temp_x][temp_y + 1] < dist)
            {
                v_dir = EAST;
                //line(frame, Point(temp_x, temp_y), Point(temp_x, temp_y + 1), Scalar(0, 255, 0), 2);
                output[c] = R;
                xo[c] = temp_x;
                yo[c] = temp_y;
                dist = wgrid[temp_x][temp_y + 1];
                temp_x += 0;
                temp_y += 1;
            }

            else if (wgrid[temp_x][temp_y - 1] < dist)
            {
                v_dir = WEST;
                //line(frame, Point(temp_x, temp_y), Point(temp_x, temp_y - 1), Scalar(0, 255, 0), 2);
                output[c] = L;
                xo[c] = temp_x;
                yo[c] = temp_y;
                dist = wgrid[temp_x][temp_y - 1];
                temp_x -= 0;
                temp_y -= 1;
            }
            else if (wgrid[temp_x + 1][temp_y] < dist)
            {
                v_dir = SOUTH;
                //line(frame, Point(temp_x, temp_y), Point(temp_x + 1, temp_y), Scalar(0, 255, 0), 2);
                output[c] = B;
                xo[c] = temp_x;
                yo[c] = temp_y;
                dist = wgrid[temp_x + 1][temp_y];
                temp_x += 1;
                temp_y += 0;
            }
            else
            {
                wgrid[temp_x][temp_y] += 2;
                var_init();
                continue;
            }
            c++;
        }


    }
}
