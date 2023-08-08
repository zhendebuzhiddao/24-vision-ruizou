# 24-vision-ruizou
实习生姓名：邹睿
核心任务
能量机关激活
1.代码思路：程序首先打开视频文件，从视频中读取每一帧图像，并分离视频的RGB通道。然后，程序使用二值化技术将视频的每个像素点变为黑白两种颜色，以便更容易地找到轮廓。接着，程序使用OpenCV库中的findContours函数查找视频中的轮廓，并对找到的轮廓进行处理，以找到最小外接矩形。最后，程序使用拟合圆函数LeastSquaresCircleFitting来拟合最小外接矩形的中心点，从而得到圆形物体的圆心和半径。
2.遇到问题和解决思路：首先是#include<opencv2/opencv.hpp>报错了，一开始认为是没有更新opencv重新下载4.7版本，然后编译不过关又把文件调整至编译目录，后面把cmakelist中添加${OpenCV_LIBS}，以及把find_package(OpenCV 4 REQUIRED)改为find_package(OpenCV REQUIRED)，然后就不报错了。接着在传video值的时候变成空值也没搞懂怎么回事然后就重新改
3.效果图：
[效果图.zip](https://github.com/zhendebuzhiddao/24-vision-ruizou/files/12291799/default.zip)
4.总结：感觉预测好难没什么思路，然后写代码出现好多bug报错，感觉自己能力还不够要通过网上查然后咨询学长才解决，还是应该多积累才能处理报错。
git仓库地址：https://github.com/zhendebuzhiddao/24-vision-ruizou
