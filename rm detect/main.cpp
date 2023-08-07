// #include<iostream>
// #include<opencv2/opencv.hpp>
// #include<opencv2/highgui.hpp>
// using namespace cv;

// int main()
// {
// 	// VideoCapture capture;
// 	// Mat frame;
// 	// frame = capture("/home/buzhidao/rune-detect.avi");
// 	// if (!capture.isOpened())
// 	// {
// 	// 	printf("can not open ...\n");
// 	// 	return -1;
// 	// }
// 	// namedWindow("output", WINDOW_AUTOSIZE);

// 	// while (capture.read(frame))
// 	// {
// 	// 	imshow("output", frame);
// 	// 	waitKey(60);
// 	// }
// 	// capture.release();
// 	// return 0;
// 	Mat src;
// 	VideoCapture capture("/home/buzhidao/rune-detect.avi");
// 	 while (capture.isOpened())
// 	 {
// 		capture.read(src);
// 		imshow("src",src);
// 		waitKey(10);
// 		//capture.release();
// 	 	//return 0;
// 	 }
	 
//  }
// #include <iostream>
// #include <string>
// #include <opencv2/opencv.hpp>

// using namespace std;

// typedef struct armor_point{
//     cv::Point point;
//     double dis;
// }armor_point;

// void drawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness)
// {
//     cv::Point2f Vertex[4];
//     rect.points(Vertex);
//     for(int i = 0 ; i < 4 ; i++)
//     {
//         cv::line(img , Vertex[i] , Vertex[(i + 1) % 4] , color , thickness);
//     }
// }
// double distance(cv::Point a,cv::Point b)
// {
//     return sqrt((a.x -b.x)*(a.x -b.x) + (a.y -b.y)*(a.y -b.y));
// }

// int main()
// {
//     string video_path = "/home/buzhidao/rune-detect.avi";
//     cv::VideoCapture cap;
//     cap.open(video_path);


//     cv::VideoWriter writer;
//     int coder = cv::VideoWriter::fourcc('M','J','P','G');//选择编码格式
//     double fps=25.0;//设置视频帧率
//     string filename="rune-detect.avi";//保存的视频文件名称
//     writer.open(filename,coder,fps,cv::Size(1350,1080),true);//创建保存视频文件的视频流
//     if(!writer.isOpened()){
//         cout<<"打开视频文件失败，请确认是否为合法输入"<<endl;
//         return -1;
//     }

//     while(true){
//         cv::Mat src_frame;
//         cap >> src_frame;
//         cv::resize(src_frame,src_frame,cv::Size(640,480));

//         vector<cv::Mat> bgr_images;
//         cv::split(src_frame,bgr_images);
//         cv::Mat b_src_img = bgr_images[0];
//         cv::blur(b_src_img,b_src_img,cv::Size(3,3));
       
//         cv::Mat threshold_img;
//         cv::threshold(b_src_img,threshold_img,130,255,cv::THRESH_BINARY);
       
//         vector<vector<cv::Point>> contours;
//         cv::findContours(threshold_img,contours,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//         // cv::drawContours(src_frame,contours,-1,cv::Scalar(0,0,255));
       
//         cv::Point r_center; // R 的中心点
//         vector<cv::RotatedRect> contours_min_rects;//所有轮廓的最小外接矩形
//         vector<cv::RotatedRect> armor_min_rects;
//         vector<cv::RotatedRect> target_min_rects;
//         vector<cv::RotatedRect> r_min_rects; //R

//         int r_index = -1;
//         // int cnt = 0;
//         for (unsigned int contour_index = 0; contour_index < contours.size(); contour_index++) {
//         //     //寻找最小旋转矩形,放进容器，顺便将面积过大的剔除,长宽比悬殊的剔除
//             cv::RotatedRect minrect = minAreaRect(contours[contour_index]);
//             cv::Rect rect = boundingRect(contours[contour_index]);

//             if (minrect.size.area() <= 6000.0 && minrect.size.area()>150) {
//                 float width;
//                 float height;
//                 //判断长宽比
//                 if (minrect.size.width > minrect.size.height){
//                     width = minrect.size.width;
//                     height = minrect.size.height;
//                 } else {
//                     width = minrect.size.height;
//                     height = minrect.size.width;
//                 }
//                 if (width / height < 5) {
//                     contours_min_rects.push_back(minrect);
//                     if(minrect.size.area() > 200 && minrect.size.area() < 350 && minrect.center.y > 100) { // find R
//                         if(height / width > 0.85){
//                             // R_minRects.push_back(minrect);
//                             r_center = minrect.center;
//                             cv::circle(src_frame,minrect.center,15,cv::Scalar(5,255,100));
//                             r_index = contour_index;
//                             // std::cout<<cnt++<<std::endl;    
//                         }
//                     } else {
//                         if(minrect.size.area() > 300 && minrect.size.area() < 4350 && (height / width) < 0.7) {
//                             armor_min_rects.push_back(minrect);
//                         }    
//                     }
//                 }  
//             }
//         }
//         bool find_ok = false;
//         for (int i = 0;i<armor_min_rects.size()-1;i++){
//             for (int j = i+1;j<armor_min_rects.size();j++){
//                 double dis = distance(armor_min_rects[i].center,armor_min_rects[j].center);
//                 if(dis<100){
//                     target_min_rects.push_back(armor_min_rects[i]);
//                     target_min_rects.push_back(armor_min_rects[j]);
//                     find_ok = true;
//                     break;
//                 }
//                 // std::cout<<dis<<std::endl;
//             }  
//             if (find_ok){
//                 break;
//             }    
//         }
//         if(target_min_rects.size() != 2){
//             continue;
//         } else {
//             cv::RotatedRect rrect_in; //= target_minRects[0];
//             cv::RotatedRect rrect_out;// = target_minRects[1];
//             double dis1 = distance(r_center,target_min_rects[0].center);
//             double dis2 = distance(r_center,target_min_rects[1].center);
//             if (dis1 > dis2){
//                 rrect_in = target_min_rects[1];
//                 rrect_out = target_min_rects[0];
//             } else {
//                 rrect_in = target_min_rects[0];
//                 rrect_out = target_min_rects[1];
//             }
//             drawRotatedRect(src_frame,rrect_in,cv::Scalar(0,250,0),1);
//             drawRotatedRect(src_frame,rrect_out,cv::Scalar(0,0,255),1);

//             cv::Point target_center = cv::Point((int)((rrect_in.center.x + rrect_out.center.x)/2),(int)((rrect_in.center.y + rrect_out.center.y)/2));
//             cv::circle(src_frame,target_center,5,cv::Scalar(0,0,255),-1);
//             cv::Point2f in_vet_points[4];
//             cv::Point2f out_vet_points[4];
//             rrect_in.points(in_vet_points);
//             rrect_out.points(out_vet_points);
//             vector<armor_point> armor_points;
//             for(int i = 0;i<4;i++){
//                 armor_point point;
//                 point.point = in_vet_points[i];
//                 point.dis = distance(target_center,in_vet_points[i]);
//                 armor_points.push_back(point);
//             }
//             for(int i = 0;i<4;i++){
//                 armor_point point;
//                 point.point = out_vet_points[i];
//                 point.dis = distance(target_center,out_vet_points[i]);
//                 armor_points.push_back(point);
//             }
//             sort(armor_points.begin(),armor_points.end(),[](armor_point a,armor_point b){return a.dis < b.dis;});
//             for(int i = 0;i<4;i++)
//             {
//                 cv::circle(src_frame,armor_points[i].point,3,cv::Scalar(255,0,255),-1);
//             }
//             int buff_run_radius = (int)distance(r_center,target_center);
//             cv::circle(src_frame,r_center,buff_run_radius,cv::Scalar(55,110,255),1);
//         }
//          cv::resize(src_frame,src_frame,cv::Size(1350,1080));
//          writer.write(src_frame);//把图像写入视频流
//         cv::imshow("src_img",src_frame);
//          cv::imshow("threshold_img",threshold_img);
//         if (cv::waitKey(0) == 'q'){
//             break;
//         }
       
//      }
//     // writer.release();
//     //cap.release();
//     //return 0;
// }
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
}
// #include <iostream>
// #include <opencv2/opencv.hpp>
// #include <math.h>
// #include <ctime>

// using namespace std;
// using namespace cv;

// #define multiple  1.5/*倍率，换算目标点所用*/
// #define min -1/*极小量*/
// #define max 10000 /*极大量*/
// #define pointSetnumeber 5/*点集大小*/
// #define speedSetnumber 2/*速度集合大小*/
// #define blue 0/*蓝色*/
// #define red 1/*红色*/
// #define change 1/*偏移*/
// #define retain 0/*保持*/

// /*________________变量区_________________*/
// Point2f pointSet[pointSetnumeber];/*定义点集,存放目标点*/
// int pointNumber = 0;/*配合pointSet[pointSetnumeber]，存放点*/
// int runNumber = 0;/*存放目标点的次数，次数达标开始预测*/
// float speed;
// float acceleratedSpeed;/*速度，加速度*/
// float speedSet[speedSetnumber];/*速度集合*/
// int speedNumber = 0;/*配合speedSet[speedSetnumber]，存放速度*/
// float predictdistance;
// Point2f predictPoint;/*定义预测距离和预测点*/
// float lastDistance = 0;/*初始化距离*/
// int frame;/*帧数*/
// int color;/*控制识别颜色，0代表蓝色，1代表红色*/

// int minId;
// double minArea = max;/*存放中心处面积*/
// Point2f center;  /*定义外接圆中心坐标*/
// float radius;  /*定义外接圆半径*/
// Point2f oldcenter;   /*定义旧外接圆圆心坐标*/
// Point2f newcenter;  /*定义新外接圆中心坐标*/
// float newradius;  /*定义新外接圆半径*/

// int maxId;
// double maxArea = min;/*存放筛选后最大面积轮廓*/
// float referenceR;/*半径参考长度*/
// Point2f rectMid;/*半径参考长度所在轮廓几何中心*/
// Point2f target;/*目标点*/
// int state;/*是否偏移*/
// /*-------------------------------------------------------------------------*/

// /*________________函数声明___________________*/
// float distance(Point2f lastPoint, Point2f presentPoint);/*计算两点间的距离*/
// /*---------------------------------------------------------------------*/

// /********************************************************** **********************
// 文件名：wind.cpp
// 介绍:
// 作者：
// 版本：2023.1.15.01
// 完成时间：2023.1.15
// * ********************************************************* ***********************/
// float distance(Point2f lastPoint, Point2f presentPoint) {
// float distance;
// distance = sqrt((presentPoint.x - lastPoint.x) * (presentPoint.x - lastPoint.x) + (presentPoint.y - lastPoint.y) * (presentPoint.y - lastPoint.y));
// return distance;
// }
// int main()
// {
// VideoCapture cap("/home/buzhidao/rune-detect.avi");
// Mat image;
// color = blue;/*蓝色*/
// for (;;) {
// cap.read(image);
// /*开始处理图像*/
// clock_t start = clock();
// /*改变大小，提高帧率*/
// resize(image, image, Size(640, 480));
// /*测试效果展示*/
// Mat test;
// image.copyTo(test);
// /*容器，存放分离通道后的图像*/
// vector<Mat> imgChannels;
// split(image, imgChannels);
// /*
// 红色
// Mat midimage = imgChannels.at(2) - imgChannels.at(0);
// imshow("1", midimage);
// */
// /*蓝色*/
// Mat blueImage = imgChannels.at(0) - imgChannels.at(2);
// Mat binaryImage;
// Mat binaryImagecricle;
// /*二值化*/
// threshold(blueImage, binaryImagecricle, 170, 255, THRESH_BINARY);
// threshold(blueImage, binaryImage, 90, 255, THRESH_BINARY);
// /*腐蚀操作*/
// Mat element = getStructuringElement(MORPH_RECT, Size(1, 1));
// Mat dstImage;
// erode(binaryImagecricle, dstImage, element);
// /*找到圆周运动的圆心——R*/
// state = retain;
// vector<vector<Point>> outlines;
// vector<Vec4i> hierarchies;
// findContours(dstImage, outlines, hierarchies, RETR_TREE, CHAIN_APPROX_NONE);
// for (int i = 0; i < outlines.size(); i++) {
// vector<Point>points;
// double area = contourArea(outlines[i]);
// /*面积排除噪声*/
// if (area < 10 || area>10000)
// continue;
// /*找到没有父轮廓的轮廓*/
// if (hierarchies[i][3] >= 0 && hierarchies[i][3] < outlines.size())
// continue;
// /*找有子轮廓的*/
// if (hierarchies[i][2] < 0 || hierarchies[i][2] >= outlines.size())
// continue;
// /*控制误差范围*/
// if (area <= minArea + 10 && area >= minArea - 20) {
// minArea = area;
// minId = i;
// continue;
// }
// /*面积最小的轮廓*/
// if (minArea >= area)
// {
// minArea = area;
// minId = i;
// }
// }
// /*防止minId不在范围内报错*/
// if(minId>=0&&minId<outlines.size())
// {
// /*画外接圆并找到圆心*/
// minEnclosingCircle(Mat(outlines[minId]), newcenter, newradius);
// /*减小抖动，误差*/
// if (distance(newcenter,center)<2) {
// }
// else {
// oldcenter = center;
// center = newcenter;
// state = change;
// }
// if (fabs(newradius - radius) < 2) {
// }
// else {
// radius = newradius;
// }
// cv::circle(test, center, radius, Scalar(0, 0, 255), 1, 8, 0);
// }
// else {
// continue;
// }
// /*膨胀操作*/
// element = getStructuringElement(0, Size(3, 3));
// Mat dilateImage;
// /*dilate最后一个数字是膨胀次数*/
// dilate(binaryImage, dilateImage, element, Point(-1, -1), 2);
// /*轮廓发现*/
// vector<vector<Point>> contours;
// vector<Vec4i> hierarchy;
// findContours(dilateImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
// for (int i = 0; i < contours.size(); i++) {
// vector<Point>points;
// double area = contourArea(contours[i]);
// /*面积排除噪声*/
// if (area < 20 || area>10000)
// continue;
// /*找到没有父轮廓的轮廓*/
// if (hierarchy[i][3] >= 0 && hierarchy[i][3] < contours.size())
// continue;
// /*找没子轮廓的*/
// if (hierarchy[i][2] >= 0 && hierarchy[i][2] < contours.size())
// continue;
// /*找面积最大的轮廓*/
// if (maxArea <= area)
// {
// maxArea = area;
// maxId = i;
// }
// /*控制误差范围*/
// if (area <= maxArea + 50 && area >= maxArea - 50) {
// maxArea = area;
// maxId = i;
// }
// std::cout << maxArea << endl;
// }
// if(maxId>=0&&maxId<contours.size()){
// /*计算矩*/
// Moments rect;
// rect = moments(contours[maxId], false);
// /*计算中心矩:*/
// Point2f rectmid;
// rectmid = Point2f(rect.m10 / rect.m00, rect.m01 / rect.m00);
// /*画出需打部位轮廓*/
// drawContours(test, contours,maxId , Scalar(0, 255, 255), 1, 8);
// /*减小抖动*/
// if (runNumber < 2) {
// referenceR = distance(rectmid, center);
// rectMid = rectmid;
// }else if (distance(rectmid, center) <= referenceR+2 && distance(rectmid, center) >= referenceR - 2 && distance(rectmid,rectMid)<0.5) {
// }
// else {
// referenceR= distance(rectmid, center);
// rectMid = rectmid;
// }
// /*画出样本部位中心点*/
// cv::circle(test, rectMid, 1, Scalar(0, 255, 255), -1, 8, 0);
// /*2：1计算需打击部位,存放*/
// /*第一象限*/
// if (rectMid.x >= center.x && rectMid.y <= center.y) {
// target = Point2f(center.x+(rectMid.x-center.x)*multiple,center.y-(center.y-rectMid.y)*multiple);
// }
// /*第二象限*/
// if (rectMid.x <= center.x && rectMid.y <= center.y) {
// target = Point2f(center.x - ( center.x- rectMid.x) * multiple, center.y - (center.y - rectMid.y) * multiple);
// }
// /*第三象限*/
// if (rectMid.x <= center.x && rectMid.y >= center.y) {
// target = Point2f(center.x - (center.x - rectMid.x) * multiple, center.y + (rectMid.y - center.y) * multiple);
// }
// /*第四象限*/
// if (rectMid.x >= center.x && rectMid.y >= center.y) {
// target = Point2f(center.x + (rectMid.x - center.x) * multiple, center.y + (rectMid.y -center.y ) * multiple);

// }
// cv::circle(test, target, 1, Scalar(0, 255, 255), -1, 8, 0);
// /*将几何中心点存入点集*/
// pointSet[pointNumber] = target;
// pointNumber++;
// /*实现新点替换旧点*/
// if (pointNumber == pointSetnumeber) {
// pointNumber = 0;
// }

// }
// else {
// continue;
// }
// /*算偏移*/
// if (state == change) {
// float xchange;
// float ychange;
// xchange = center.x - oldcenter.x;
// ychange = center.y - oldcenter.y;
// /*改变点集*/
// if (pointNumber == 0) {
// pointSet[0] = Point2f(pointSet[0].x + xchange, pointSet[0].y + ychange);
// pointSet[1] = Point2f(pointSet[1].x + xchange, pointSet[1].y + ychange);
// pointSet[2] = Point2f(pointSet[2].x + xchange, pointSet[2].y + ychange);
// pointSet[3] = Point2f(pointSet[3].x + xchange, pointSet[3].y + ychange);
// }
// if (pointNumber == 1) {
// pointSet[1] = Point2f(pointSet[1].x + xchange, pointSet[1].y + ychange);
// pointSet[2] = Point2f(pointSet[2].x + xchange, pointSet[2].y + ychange);
// pointSet[3] = Point2f(pointSet[3].x + xchange, pointSet[3].y + ychange);
// pointSet[4] = Point2f(pointSet[4].x + xchange, pointSet[4].y + ychange);
// }
// if (pointNumber == 2) {
// pointSet[2] = Point2f(pointSet[2].x + xchange, pointSet[2].y + ychange);
// pointSet[3] = Point2f(pointSet[3].x + xchange, pointSet[3].y + ychange);
// pointSet[4] = Point2f(pointSet[4].x + xchange, pointSet[4].y + ychange);
// pointSet[0] = Point2f(pointSet[0].x + xchange, pointSet[0].y + ychange);
// }
// if (pointNumber == 3) {
// pointSet[3] = Point2f(pointSet[3].x + xchange, pointSet[3].y + ychange);
// pointSet[4] = Point2f(pointSet[4].x + xchange, pointSet[4].y + ychange);
// pointSet[0] = Point2f(pointSet[0].x + xchange, pointSet[0].y + ychange);
// pointSet[1] = Point2f(pointSet[1].x + xchange, pointSet[1].y + ychange);
// }
// if (pointNumber == 4) {
// pointSet[4] = Point2f(pointSet[4].x + xchange, pointSet[4].y + ychange);
// pointSet[0] = Point2f(pointSet[0].x + xchange, pointSet[0].y + ychange);
// pointSet[1] = Point2f(pointSet[1].x + xchange, pointSet[1].y + ychange);
// pointSet[2] = Point2f(pointSet[2].x + xchange, pointSet[2].y + ychange);
// }
// }
// /*预测*/
// if (runNumber > pointSetnumeber) {
// int i = pointNumber - 1;//取最新的点算速度
// int number1 = i;
// if (number1 < 0) {
// number1 += pointSetnumeber;
// }
// int number2 = i - 1;
// if (number2 < 0) {
// number2 += pointSetnumeber;
// }
// int number3 = i - 3;
// if (number3 < 0) {
// number3 += pointSetnumeber;
// }
// int number4 = i - 4;
// if (number4 < 0) {
// number4 += pointSetnumeber;
// }
// /*取最近四点，算速度，求加速度*/
// speed = distance(pointSet[number1], pointSet[number2]) * frame;
// speedSet[0] = speed;
// speed = distance(pointSet[number3], pointSet[number4]) * frame;
// speedSet[1] = speed;
// acceleratedSpeed = fabs((speedSet[0] - speedSet[1]) * frame);
// /* X = V0T + 1 / 2AT'2，通过距离公式，算预测的打击点距离 */
// predictdistance = 4.5 * speedSet[0] / frame + 1 / 2 * acceleratedSpeed / frame / frame * 18;
// /*算出预测时x, y需增加值的比值*/
// float xRatio, yRatio;
// xRatio = fabs(pointSet[number1].x - pointSet[number2].x) / distance(pointSet[number1], pointSet[number2]);
// yRatio = fabs(pointSet[number1].y - pointSet[number2].y) / distance(pointSet[number1], pointSet[number2]);
// /*第一象限内  顺  三逆*/
// if (pointSet[number1].x >= pointSet[number2].x && pointSet[number1].y >= pointSet[number2].y) {
// predictPoint = Point2f(pointSet[number1].x + predictdistance * xRatio, pointSet[number1].y + predictdistance * yRatio);
// }
// /*第二象限内  顺  四逆*/
// if (pointSet[number1].x >= pointSet[number2].x && pointSet[number1].y <= pointSet[number2].y) {
// predictPoint = Point2f(pointSet[number1].x + predictdistance * xRatio, pointSet[number1].y - predictdistance * yRatio);
// }
// /*第三象限内  顺  一逆*/
// if (pointSet[number1].x <= pointSet[number2].x && pointSet[number1].y <= pointSet[number2].y) {
// predictPoint = Point2f(pointSet[number1].x - predictdistance * xRatio, pointSet[number1].y - predictdistance * yRatio);
// }
// /*第四象限内  顺  二逆*/
// if (pointSet[number1].x <= pointSet[number2].x && pointSet[number1].y >= pointSet[number2].y) {
// predictPoint = Point2f(pointSet[number1].x - predictdistance * xRatio, pointSet[number1].y + predictdistance * yRatio);
// }
// /*减少预测抖动*/
// lastDistance = predictdistance;
// /*向轨迹拟合路径*/
// /*画圆*/
// cv::circle(test, center, distance(center,target)+3, Scalar(0, 255, 255), 1, 8, 0);
// /*预测点像圆弧靠拢*/
// /*第一象限*/
// if (predictPoint.x >= center.x && predictPoint.y <= center.y) {
// predictPoint = Point2f(center.x + (predictPoint.x - center.x) * distance(center, target)/distance(center,predictPoint), center.y - (center.y - predictPoint.y) * distance(center, target) / distance(center, predictPoint));
// }
// /*第二象限*/
// if (predictPoint.x <= center.x && predictPoint.y <= center.y) {
// predictPoint = Point2f(center.x - (center.x - predictPoint.x) * distance(center, target) / distance(center, predictPoint), center.y - (center.y - predictPoint.y) * distance(center, target) / distance(center, predictPoint));
// }
// /*第三象限*/
// if (predictPoint.x <= center.x && predictPoint.y >= center.y) {
// predictPoint = Point2f(center.x - (center.x - predictPoint.x) * distance(center, target) / distance(center, predictPoint), center.y + (predictPoint.y - center.y) * distance(center, target) / distance(center, predictPoint));
// }
// /*第四象限*/
// if (predictPoint.x >= center.x && predictPoint.y >= center.y) {
// predictPoint = Point2f(center.x + (predictPoint.x - center.x) * distance(center, target) / distance(center, predictPoint), center.y + (predictPoint.y - center.y) * distance(center, target) / distance(center, predictPoint));
// }
// /*画出预测点*/
// cv::circle(test, predictPoint, 2, Scalar(0, 0, 255), -1, 8, 0);
// }
// cv::imshow("1",test);
// cv::imshow("2", dilateImage);
// runNumber++;
// clock_t end = clock();
// frame = 1 / (double)(end - start) * CLOCKS_PER_SEC;
// std::cout << frame << "帧" << endl;
// cv::waitKey(1);
// }
// }
/*
 * @Descripttion: 
 * @version: 
 * @Author: tjk
 * @Date: 2023-01-31 17:01:02
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2023-02-01 17:04:12
 */
// #include <iostream>
// #include <string>
// #include <opencv2/opencv.hpp>
// // #include <opencv2/opencv.hpp>
// using namespace std;

// typedef struct armor_point{
//     cv::Point point;//?
//     double dis;
// }armor_point;

// void drawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness)
// {
//     cv::Point2f Vertex[4];
//     rect.points(Vertex);
//     for(int i = 0 ; i < 4 ; i++)
//     {
//         cv::line(img , Vertex[i] , Vertex[(i + 1) % 4] , color , thickness);
//     }
// }
// double distance(cv::Point a,cv::Point b)
// {
//     return sqrt((a.x -b.x)*(a.x -b.x) + (a.y -b.y)*(a.y -b.y));
// }

// int main()
// {
//     string video_path = "/home/idriver/tjk/project/rm/buff_demo/blue.mp4"; 
//     cv::VideoCapture cap;
//     cap.open(video_path);


//     // cv::VideoWriter writer;
//     // int coder = cv::VideoWriter::fourcc('M','J','P','G');//选择编码格式
//     // double fps=25.0;//设置视频帧率
//     // string filename="live.avi";//保存的视频文件名称
//     // writer.open(filename,coder,fps,cv::Size(1350,1080),true);//创建保存视频文件的视频流
//     // if(!writer.isOpened()){
//     //     cout<<"打开视频文件失败，请确认是否为合法输入"<<endl;
//     //     return -1;
//     // }

//     while(true){
//         cv::Mat src_frame;
//         cap >> src_frame;
//         cv::resize(src_frame,src_frame,cv::Size(640,480));

//         vector<cv::Mat> bgr_images;
//         cv::split(src_frame,bgr_images);
//         cv::Mat b_src_img = bgr_images[0];
//         cv::blur(b_src_img,b_src_img,cv::Size(3,3));
        
//         cv::Mat threshold_img;
//         cv::threshold(b_src_img,threshold_img,130,255,cv::THRESH_BINARY);
        
//         vector<vector<cv::Point>> contours;
//         cv::findContours(threshold_img,contours,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//         // cv::drawContours(src_frame,contours,-1,cv::Scalar(0,0,255));
        
//         cv::Point r_center; // R 的中心点
//         vector<cv::RotatedRect> contours_min_rects;//所有轮廓的最小外接矩形
//         vector<cv::RotatedRect> armor_min_rects;
//         vector<cv::RotatedRect> target_min_rects;
//         vector<cv::RotatedRect> r_min_rects; //R

//         int r_index = -1;
//         // int cnt = 0;
//         for (unsigned int contour_index = 0; contour_index < contours.size(); contour_index++) {
//         //     //寻找最小旋转矩形,放进容器，顺便将面积过大的剔除,长宽比悬殊的剔除
//             cv::RotatedRect minrect = minAreaRect(contours[contour_index]);
//             cv::Rect rect = boundingRect(contours[contour_index]);

//             if (minrect.size.area() <= 6000.0 && minrect.size.area()>150) {
//                 float width;
//                 float height;
//                 //判断长宽比
//                 if (minrect.size.width > minrect.size.height){
//                     width = minrect.size.width;
//                     height = minrect.size.height;
//                 } else {
//                     width = minrect.size.height;
//                     height = minrect.size.width;
//                 }
//                 if (width / height < 5) {
//                     contours_min_rects.push_back(minrect);
//                     if(minrect.size.area() > 200 && minrect.size.area() < 350 && minrect.center.y > 100) { // find R
//                         if(height / width > 0.85){
//                             // R_minRects.push_back(minrect);
//                             r_center = minrect.center;
//                             cv::circle(src_frame,minrect.center,15,cv::Scalar(5,255,100));
//                             r_index = contour_index;
//                             // std::cout<<cnt++<<std::endl;    
//                         }
//                     } else {
//                         if(minrect.size.area() > 300 && minrect.size.area() < 4350 && (height / width) < 0.7) {
//                             armor_min_rects.push_back(minrect);
//                         }     
//                     }
//                 }   
//             }
//         }
//         bool find_ok = false;
//         for (int i = 0;i<armor_min_rects.size()-1;i++){
//             for (int j = i+1;j<armor_min_rects.size();j++){
//                 double dis = distance(armor_min_rects[i].center,armor_min_rects[j].center);
//                 if(dis<100){
//                     target_min_rects.push_back(armor_min_rects[i]);
//                     target_min_rects.push_back(armor_min_rects[j]);
//                     find_ok = true;
//                     break;
//                 }
//                 // std::cout<<dis<<std::endl;
//             }   
//             if (find_ok){
//                 break;
//             }    
//         }
//         if(target_min_rects.size() != 2){
//             continue;
//         } else {
//             cv::RotatedRect rrect_in; //= target_minRects[0];
//             cv::RotatedRect rrect_out;// = target_minRects[1];
//             double dis1 = distance(r_center,target_min_rects[0].center);
//             double dis2 = distance(r_center,target_min_rects[1].center);
//             if (dis1 > dis2){
//                 rrect_in = target_min_rects[1];
//                 rrect_out = target_min_rects[0];
//             } else {
//                 rrect_in = target_min_rects[0];
//                 rrect_out = target_min_rects[1];
//             }
//             drawRotatedRect(src_frame,rrect_in,cv::Scalar(0,250,0),1);
//             drawRotatedRect(src_frame,rrect_out,cv::Scalar(0,0,255),1);

//             cv::Point target_center = cv::Point((int)((rrect_in.center.x + rrect_out.center.x)/2),(int)((rrect_in.center.y + rrect_out.center.y)/2));
//             cv::circle(src_frame,target_center,5,cv::Scalar(0,0,255),-1);
//             cv::Point2f in_vet_points[4];
//             cv::Point2f out_vet_points[4];
//             rrect_in.points(in_vet_points);
//             rrect_out.points(out_vet_points);
//             vector<armor_point> armor_points; 
//             for(int i = 0;i<4;i++){
//                 armor_point point;
//                 point.point = in_vet_points[i];
//                 point.dis = distance(target_center,in_vet_points[i]);
//                 armor_points.push_back(point);
//             }
//             for(int i = 0;i<4;i++){
//                 armor_point point;
//                 point.point = out_vet_points[i];
//                 point.dis = distance(target_center,out_vet_points[i]);
//                 armor_points.push_back(point);
//             }
//             sort(armor_points.begin(),armor_points.end(),[](armor_point a,armor_point b){return a.dis < b.dis;});
//             for(int i = 0;i<4;i++)
//             {
//                 cv::circle(src_frame,armor_points[i].point,3,cv::Scalar(255,0,255),-1);
//             }
//             int buff_run_radius = (int)distance(r_center,target_center);
//             cv::circle(src_frame,r_center,buff_run_radius,cv::Scalar(55,110,255),1);
//         }
//         // cv::resize(src_frame,src_frame,cv::Size(1350,1080));
//         // writer.write(src_frame);//把图像写入视频流
//         cv::imshow("src_img",src_frame);
//         // cv::imshow("threshold_img",threshold_img);
//         if (cv::waitKey(0) == 'q'){
//             break;
//         }
        
//     }
//     // writer.release();
//     cap.release();
//     return 0;
// }
