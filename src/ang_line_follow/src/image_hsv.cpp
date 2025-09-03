#include <ros/ros.h>                      // 引入ROS的头文件
#include <image_transport/image_transport.h> // 引入图像传输模块的头文件
#include <cv_bridge/cv_bridge.h>           // 引入CV桥接模块的头文件
#include <sensor_msgs/image_encodings.h>   // 引入传感器消息的图像编码头文件
#include <opencv2/imgproc/imgproc.hpp>     // 引入OpenCV图像处理模块的头文件
#include <opencv2/highgui/highgui.hpp>     // 引入OpenCV图形用户界面模块的头文件
#include <dynamic_reconfigure/server.h>    // 引入动态重配置服务器的头文件
#include <ang_line_follow/HSVThresholdConfig.h> // 引入自定义的HSV阈值配置文件

using namespace cv;                        // 使用OpenCV命名空间
using namespace std;                       // 使用标准命名空间

// 定义HSV色彩空间中的低高阈值
static int iLowH = 0;
static int iHighH = 179;

static int iLowS = 0; 
static int iHighS = 120;

static int iLowV = 0;
static int iHighV = 255;

// 图像发布器
image_transport::Publisher rgb_pub;
image_transport::Publisher hsv_pub;
image_transport::Publisher result_pub;

// 动态重配置回调函数
void dynamicReconfigureCallback(ang_line_follow::HSVThresholdConfig &config, uint32_t level) {
    iLowH = config.low_H;
    iHighH = config.high_H;
    iLowS = config.low_S;
    iHighS = config.high_S;
    iLowV = config.low_V;
    iHighV = config.high_V;
}

// 图像回调函数
void Cam_RGB_Callback(const sensor_msgs::ImageConstPtr& msg)
{
    // 创建一个cv_bridge的CvImage指针
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        // 将ROS的图像消息转换为OpenCV的图像格式
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        // 捕获并打印异常
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 获取原始图像
    Mat imgOriginal = cv_ptr->image;
    
    // 将RGB图像转换到HSV色彩空间
    Mat imgHSV;
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

    // 在HSV色彩空间中进行直方图均衡化
    vector<Mat> hsvSplit;
    split(imgHSV, hsvSplit);
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, imgHSV);

    // 对图像进行阈值处理，仅保留HSV值在指定范围内的像素
    Mat imgThresholded;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); 

    // 进行开运算，去除小的噪点
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

    // 进行闭运算，连接相邻的区域
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

    // 相关二值化之后的变量
    int nTargetX = 0;
    int nTargetY = 0;
    int nPixCount = 0;
    int nImgWidth = imgThresholded.cols;
    int nImgHeight = imgThresholded.rows;

    // 发布原始RGB图像
    sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgOriginal).toImageMsg();
    rgb_pub.publish(rgb_msg);

    // 发布HSV图像
    sensor_msgs::ImagePtr hsv_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgHSV).toImageMsg();
    hsv_pub.publish(hsv_msg);

    // 发布处理后的结果图像
    sensor_msgs::ImagePtr result_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgThresholded).toImageMsg();
    result_pub.publish(result_msg);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "image_hsv");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 设置动态重配置服务器
    dynamic_reconfigure::Server<ang_line_follow::HSVThresholdConfig> server;
    dynamic_reconfigure::Server<ang_line_follow::HSVThresholdConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    // 订阅Kinect的RGB图像
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber rgb_sub = it.subscribe("/ucar_camera/image_raw", 1, Cam_RGB_Callback);

    // 创建图像发布器
    rgb_pub = it.advertise("/image_hsv/rgb", 1);
    hsv_pub = it.advertise("/image_hsv/hsv", 1);
    result_pub = it.advertise("/image_hsv/result", 1);

    // 设置循环频率
    ros::Rate loop_rate(30);

    // 主循环
    while(ros::ok())
    {
        // 处理ROS的事件
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}