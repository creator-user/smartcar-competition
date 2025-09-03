#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>
using namespace std;
using namespace cv;

Mat Centre_Line;
cv::Mat croppedFrame;
cv::Mat flipped_image;

int C_x = 0;
bool is_stop=0;
ros::Publisher cmdVelPub;
ros::Publisher task_flag_pub;
double current_y = 0.0;
double current_x = 0.0;
int task_flag = 0;
Mat color_img;
void PrintBinaryImageToFile(const Mat& binary_img, const string& filename) 
{
    ofstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }

    for (int i = 0; i < binary_img.rows; ++i) {
        for (int j = 0; j < binary_img.cols; ++j) {
            uchar pixel_value = binary_img.at<uchar>(i, j);
            if (pixel_value == 255) {
                file << "1";
            } else if (pixel_value == 0) {
                file << " ";
            }
        }
        file << endl;
    }

    file.close();
}

Mat To_GRAY(const Mat& img) {
    vector<Mat> bgr_channels(3);
    split(img, bgr_channels);
    Mat b = bgr_channels[0];
    Mat g = bgr_channels[1];
    Mat r = bgr_channels[2];

    Mat white_mask = Mat::zeros(b.size(), b.type());

    for (int i = 0; i < img.rows; ++i) {
        for (int j = 0; j < img.cols; ++j) {
            if (r.at<uchar>(i, j) >= 210 && g.at<uchar>(i, j) >= 210 && b.at<uchar>(i, j) >= 210) {
                white_mask.at<uchar>(i, j) = 255;
            }
        }
    }

    white_mask.colRange(0, 2).setTo(255);
    white_mask.colRange(img.cols - 2, img.cols).setTo(255);
    PrintBinaryImageToFile(white_mask, "/home/ucar/Desktop/ucar/src/darknet_ros/darknet_ros/detection/pixel");
    return white_mask;
}

int CountWhitePixelsInRow(const Mat& binary_img, int row) {
    if (row < 0 || row >= binary_img.rows) {
        cerr << "Row index out of bounds" << endl;
        return -1;
    }

    int count = 0;
    for (int j = 0; j < binary_img.cols; ++j) {
        if (binary_img.at<uchar>(row, j) == 255) {
            ++count;
        }
    }
    return count;
}

int Centre_line(Mat binary_img)
{
    Centre_Line = Mat::zeros(binary_img.size(), CV_8UC1);

    vector<Point> Centre_Point;
    vector<Point> Left_Point;
    vector<Point> Right_Point;
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    int centre_img_y = binary_img.cols / 2;
    int w = binary_img.cols;
    int h = binary_img.rows;
    int descend = 50;
    ROS_INFO("cols/2:%d", binary_img.cols / 2);
    int centre_x = 0, centre_y = 0;
    int white_pixel_nums=0;
    for (int i = binary_img.rows; i > int(h * 0.6); --i) 
    {
        x1 = 0, y1 = 0, x2 = 0, y2 = 0;

        for (int j = centre_img_y; j > 0; --j) 
        {
            uchar pixel_value_1 = binary_img.at<uchar>(i, j);
            uchar pixel_value_2 = binary_img.at<uchar>(i, j - 1);
            if (pixel_value_1 == 255 && pixel_value_2 == 0) 
            {
                x1 = i;
                y1 = j;
                Left_Point.push_back({ y1, x1 });
                break;
            }
        }

        for (int j = centre_img_y; j < binary_img.cols - 1; ++j) {
            uchar pixel_value_1 = binary_img.at<uchar>(i, j);
            uchar pixel_value_2 = binary_img.at<uchar>(i, j + 1);
            if (pixel_value_1 == 255 && pixel_value_2 == 0) 
            {
                x2 = i;
                y2 = j;
                Right_Point.push_back({ y2, x2 });
                break;
            }
        }

        if (y2 == 0)
        {
            y2 = binary_img.cols;
            Right_Point.push_back({ y2,i });
        }
        if (y1 == 0)
        {
            Right_Point.push_back({ y1, i });
        }

        centre_y = (y1 + y2) / 2;
        C_x = centre_y;
        centre_x = i;
        if (i >= h - descend)
        {
            for (int i = 0; i <= 6; i++)
            {
                Centre_Point.push_back(Point(centre_y + i, centre_x));
            }
            for (int i = 6; i <= 0; i--)
            {
                Centre_Point.push_back(Point(centre_y - i, centre_x));
            }
            centre_img_y = binary_img.cols / 2;
        }
        else
        {
            for (int i = 0; i <= 6; i++)
            {
                Centre_Point.push_back(Point(centre_y + i, centre_x));
            }
            for (int i = 6; i <= 0; i--)
            {
                Centre_Point.push_back(Point(centre_y + i, centre_x));
            }

            centre_img_y = centre_y;

        }
    }
    for(int i=int(h*0.78);i<=h;++i)
    {
      white_pixel_nums=max(CountWhitePixelsInRow(binary_img, i),white_pixel_nums);
    }
    
    
    cvtColor(binary_img, color_img, COLOR_GRAY2BGR);
/*
    for (size_t i = 1; i < Left_Point.size(); ++i)
    {
        circle(color_img, Left_Point[i], 2, Scalar(0, 255, 0), FILLED);
    }

    for (size_t i = 1; i < Right_Point.size(); ++i)
    {
        circle(color_img, Right_Point[i], 2, Scalar(0, 255, 0), FILLED);
    }

    for (size_t i = 1; i < Centre_Point.size(); ++i) {
        circle(color_img, Centre_Point[i], 2, Scalar(0, 255, 255), FILLED);
    }
*/
    return white_pixel_nums;
}

void publishCommand(double offset,bool is_stop) 
{    
    geometry_msgs::Twist cmdVel;
    if(!is_stop)
    {
      
      double maxAngularSpeed = 1.773;
      double maxLinearSpeed = 0.225;
      double angularSpeed = maxAngularSpeed * offset;
      double linearSpeed = maxLinearSpeed * (1 - fabs(offset));
      cmdVel.angular.z = angularSpeed;
      cmdVel.linear.x = linearSpeed;
      ROS_INFO("linear:%.2lf,w:%.2lf", linearSpeed, angularSpeed);
    }
    else
    {
        cmdVel.angular.z = 0;
        cmdVel.linear.x = 0;
        std_msgs::Int8 flag_msg;
        flag_msg.data = 1;
        task_flag_pub.publish(flag_msg);
    }
    cmdVelPub.publish(cmdVel);
   
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    //ROS_INFO("imageCallback triggered");
    cv_bridge::CvImagePtr cv_ptr;
    //if (task_flag == 0) return;
    try {
        if (msg->encoding == "mono8" || msg->encoding == "bgr8" || msg->encoding == "rgb8") {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        else if (msg->encoding == "bgra8") {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        else if (msg->encoding == "rgba8") {
            cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
        }
        else if (msg->encoding == "mono16") {
            ROS_WARN_ONCE("Converting mono16 images to mono8");
            cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        }
        else {
            ROS_ERROR("Image message encoding provided is not mono8, mono16, bgr8, bgra8, rgb8 or rgba8.");
            return;
        }
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cv_ptr && !cv_ptr->image.empty()) {
        //ROS_INFO("Image successfully converted");
        flip(cv_ptr->image, flipped_image, 1);
        cv::resize(flipped_image, croppedFrame, cv::Size(73, 45));
    }
    else {
        ROS_WARN("Converted image is empty.");
    }
}

void taskFlagCallback(const std_msgs::Int8::ConstPtr& msg) {
    task_flag = msg->data;
    ROS_INFO("task_flag=%d\n", task_flag);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_y = msg->pose.pose.position.y;
    current_x = msg->pose.pose.position.x;
    //ROS_INFO("x=%.2f, y=%.2f", current_x, current_y);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "xunxian");
    ros::NodeHandle nh;

    cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    task_flag_pub = nh.advertise<std_msgs::Int8>("task_flag", 1);
    ros::Subscriber task_flag_sub = nh.subscribe("task_flag", 1, taskFlagCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("ucar_camera/image_raw", 1, imageCallback);

    ros::Rate rate(10);
    while (ros::ok()) {
    /*
        if (current_x > 1.8 && current_y < 0.30 && task_flag == 4) {
            std_msgs::Int8 flag_msg;
            flag_msg.data = 1;
            task_flag_pub.publish(flag_msg);
            geometry_msgs::Twist cmdVel;
            cmdVel.linear.x = 0.0;
            cmdVel.angular.z = 0.0;
            cmdVelPub.publish(cmdVel);
        }
      */
        
        if (!croppedFrame.empty() /*&& task_flag == 4*/) {
            // ROS_INFO("FFFF");
            int w_crop = croppedFrame.cols;
            int h_crop = croppedFrame.rows;
            ROS_INFO("Width_crop:%d Height_crop:%d", w_crop, h_crop);

            Mat binary_img = To_GRAY(croppedFrame);
            int white_pixel_nums = Centre_line(binary_img);
            ROS_INFO("binary_img_0.4:%d",int(binary_img.cols*0.4));
            if(white_pixel_nums>=int(binary_img.cols*0.4))
            {
                is_stop=1;
            }
            //imshow("binary_img",color_img);
            //waitKey(0);
            double offset = static_cast<double>(C_x - (w_crop / 2)) / (w_crop / 2);
            ROS_INFO("offset:%.2lf,is_stop:%d,white_pixel_nums:%d", offset,is_stop,white_pixel_nums);
            publishCommand(offset,is_stop);
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
