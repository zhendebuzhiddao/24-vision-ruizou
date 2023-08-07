#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int LeastSquaresCircleFitting(vector<cv::Point2d> &m_Points, cv::Point2d &Centroid, double &dRadius);

int main()
{
    
    VideoCapture video("/home/buzhidao/rune-detect.avi");//打开视频
    if (!video.isOpened())
    {
    cout << "打开失败" << endl;
	    //     // cv::VideoWriter writer;
//     // int coder = cv::VideoWriter::fourcc('M','J','P','G');//选择编码格式
//     // double fps=25.0;//设置视频帧率
//     // string filename="live.avi";//保存的视频文件名称
//     // writer.open(filename,coder,fps,cv::Size(1350,1080),true);//创建保存视频文件的视频流
//     // if(!writer.isOpened()){
//     //     cout<<"打开视频文件失败，请确认是否为合法输入"<<endl;
//     //     return -1;
//     // }
}
namedWindow("识别窗口", WINDOW_AUTOSIZE);
Mat video1, video2;

vector<Point2d> points;

while (true)
{
video >> video1;
if (video1.empty())
break;
Mat video2 = video1.clone();

vector<Mat> channels;
split(video1, channels);//分离通道
threshold(channels.at(2) - channels.at(0), video1, 100, 255, THRESH_BINARY_INV);//二值化

Mat element1 = getStructuringElement(MORPH_RECT, Size(5, 5));//设置内核1
Mat element2 = getStructuringElement(MORPH_RECT, Size(25, 25));//设置内核2
morphologyEx(video1, video1, MORPH_OPEN, element1);//开运算
floodFill(video1, Point(0, 0), Scalar(0));//漫水
morphologyEx(video1, video1, MORPH_CLOSE, element2);//闭运算
vector<vector<Point>> contours;
vector<Vec4i> hierarchy;
findContours(video1, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);//找轮廓
int area[25] = { 0 };
for (int i = 0; i < hierarchy.size(); i++)
{
area[i] = contourArea(contours[i]);//计算轮廓面积
if (area[i] < 4000)
{
Point2f rect[4];
RotatedRect box1 = minAreaRect(Mat(contours[i]));//获取最小外接矩阵
circle(video2, Point(box1.center.x, box1.center.y), 5, Scalar(255, 0, 0), -1, 8);  //绘制最小外接矩形的中心点
box1.points(rect);  //把最小外接矩形四个端点复制给rect数组
for (int j = 0; j < 4; j++)
{
line(video2, rect[j], rect[(j + 1) % 4], Scalar(0, 255, 0), 2, 8);  //绘制最小外接矩形每条边
}

points.push_back(box1.center);//储存最小外接矩形中心点
Point2d c;//圆心
double r = 0;//半径
LeastSquaresCircleFitting(points, c, r);//拟合圆
circle(video2, c, r, Scalar(0, 0, 255), 2, 8);//绘制圆
circle(video2, c, 5, Scalar(255, 0, 0), -1, 8);//绘制圆心
}
}
imshow("识别窗口", video2);
waitKey(10000);
}
//return 0;
}

int LeastSquaresCircleFitting(vector<cv::Point2d> &m_Points, cv::Point2d &Centroid, double &dRadius)//拟合圆函数
{
if (!m_Points.empty())
{
int iNum = (int)m_Points.size();
if (iNum < 3)return 1;
double X1 = 0.0;
double Y1 = 0.0;
double X2 = 0.0;
double Y2 = 0.0;
double X3 = 0.0;
double Y3 = 0.0;
double X1Y1 = 0.0;
double X1Y2 = 0.0;
double X2Y1 = 0.0;
vector<cv::Point2d>::iterator iter;
vector<cv::Point2d>::iterator end = m_Points.end();
for (iter = m_Points.begin(); iter != end; ++iter)
{
X1 = X1 + (*iter).x;
Y1 = Y1 + (*iter).y;
X2 = X2 + (*iter).x * (*iter).x;
Y2 = Y2 + (*iter).y * (*iter).y;
X3 = X3 + (*iter).x * (*iter).x * (*iter).x;
Y3 = Y3 + (*iter).y * (*iter).y * (*iter).y;
X1Y1 = X1Y1 + (*iter).x * (*iter).y;
X1Y2 = X1Y2 + (*iter).x * (*iter).y * (*iter).y;
X2Y1 = X2Y1 + (*iter).x * (*iter).x * (*iter).y;
}
double C = 0.0;
double D = 0.0;
double E = 0.0;
double G = 0.0;
double H = 0.0;
double a = 0.0;
double b = 0.0;
double c = 0.0;
C = iNum * X2 - X1 * X1;
D = iNum * X1Y1 - X1 * Y1;
E = iNum * X3 + iNum * X1Y2 - (X2 + Y2) * X1;
G = iNum * Y2 - Y1 * Y1;
H = iNum * X2Y1 + iNum * Y3 - (X2 + Y2) * Y1;
a = (H * D - E * G) / (C * G - D * D);
b = (H * C - E * D) / (D * D - G * C);
c = -(a * X1 + b * Y1 + X2 + Y2) / iNum;
double A = 0.0;
double B = 0.0;
double R = 0.0;
A = a / (-2);
B = b / (-2);
R = double(sqrt(a * a + b * b - 4 * c) / 2);
Centroid.x = A;
Centroid.y = B;
dRadius = R;
return 0;
}
else
return 1;
return 0;

