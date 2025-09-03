#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <vector>
#include <locale>
#include <codecvt>
#include <unordered_map>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdlib>

using namespace cv;
using namespace std;

// 图像处理类

class ImageProcessor {
public:
    ImageProcessor() : it_(nh_) {
        // 读取配置文件
        readParameters();

        // 使用配置的话题名称订阅图像
        image_sub_ = it_.subscribe(image_topic_, 1, &ImageProcessor::imageCb, this);
        image_pub_ = it_.advertise("/img_follow", 1);
    }

    virtual void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge异常: %s", e.what());
            return;
        }

        Mat processed_image = processImage(cv_ptr->image);
        sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, processed_image).toImageMsg();
        image_pub_.publish(output_msg);
    }

    Mat processImage(const Mat& image) {

        // 将RGB图像转换到HSV色彩空间
        Mat imgHSV;
        cvtColor(image, imgHSV, COLOR_BGR2HSV);

        // 在HSV色彩空间中进行直方图均衡化
        vector<Mat> hsvSplit;
        split(imgHSV, hsvSplit);
        equalizeHist(hsvSplit[2], hsvSplit[2]);
        merge(hsvSplit, imgHSV);

        // 对图像进行阈值处理，仅保留HSV值在指定范围内的像素
        Mat grayImage;
        inRange(imgHSV, Scalar(LowH_, LowS_, LowV_), Scalar(HighH_, HighS_, HighV_), grayImage);

        
        // 进行开运算，去除小的噪点
        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(grayImage, grayImage, MORPH_OPEN, element);

        // 进行闭运算，连接相邻的区域
        morphologyEx(grayImage, grayImage, MORPH_CLOSE, element); 
        
        // 创建BGR图像以画出黄色线
        Mat colorImage;
        cvtColor(grayImage, colorImage, COLOR_GRAY2BGR);

        // 一开始初始化像素矩阵，进行边界255处理
        for (int y = 0; y < grayImage.rows; y++) {
            // 底部1处理
            if (y == grayImage.rows - 1) {
                for (int x = 2; x < grayImage.cols - 2; x++) {
                    grayImage.at<uchar>(y, x) = 0;
                }
            }
            // 注意：移除了左右边界255处理，因为改进后的代码不再需要这个处理
        }

        // 记录中心点位置的x
        unordered_map<int, int> left_points;
        unordered_map<int, int> right_points;
        // 记录中心点位置的x，一直更新保持连贯性质
        int center_x = grayImage.cols / 2;

        // 考虑范围
        int img_range = grayImage.rows - grayImage.rows / range_y_;
        
        // 判断当前那一帧是否当头了且并未直角转弯
        bool is_end = false;

        for (int y = grayImage.rows - 1; y >= img_range; y--) 
        {
            int mid = center_x;
            int leftBoundary = -1;
            int rightBoundary = -1;

            // 使用局部变量保存 center_x
            int local_center_x = center_x;
            
            if (grayImage.at<uchar>(y, local_center_x) == 255) {
                if (img_debug_output_) printUTF8("到头了！");
                is_end = true;
                if (left_points.empty() || right_points.empty()) {
                    is_end=false;
                    if (img_debug_output_) printUTF8("需要直角转弯！");

                    // 左转
                    if (left_points.empty() && right_points.size() <= turn_dist_) {
                        is_turn_left = true;
                        if (img_debug_output_) printUTF8("左转");
                    }

                    // 右转
                    else if (right_points.empty() && left_points.size() <= turn_dist_) {
                        is_turn_right = true;
                        if (img_debug_output_) printUTF8("右转");
                    }
                }
                is_turn_left = false;
                is_turn_right = false;
                break;
            }

            // 从中心向左遍历找到左边界点
            for (int x = mid; x >= 0; x--) {
                if (x + 1 < grayImage.cols && x - 1 >= 0) {
                    if (grayImage.at<uchar>(y, x) == 255 && grayImage.at<uchar>(y, x - 1) == 255 && grayImage.at<uchar>(y, x + 1) == 0) {
                        leftBoundary = x;
                        left_points[y] = x;
                        break;
                    }
                }
            }

            // 从中心向右遍历找到右边界点
            for (int x = mid; x < grayImage.cols; x++) {
                if (x + 1 < grayImage.cols && x - 1 >= 0) {
                    if (grayImage.at<uchar>(y, x) == 255 && grayImage.at<uchar>(y, x + 1) == 255 && grayImage.at<uchar>(y, x - 1) == 0) {
                        rightBoundary = x;
                        right_points[y] = x;
                        break;
                    }
                }
            }

            bool hasLeft = left_points.find(y) != left_points.end();
            bool hasRight = right_points.find(y) != right_points.end();
            if (hasLeft && hasRight) {
                center_x = (leftBoundary + rightBoundary) / 2;
                // 将中线点画出来
                colorImage.at<Vec3b>(y, center_x) = Vec3b(0, 255, 255); // 使用黄色
            }
            else if (!hasLeft && hasRight) {
                center_x = (rightBoundary + 1) / 2;
                // 将中线点画出来
                colorImage.at<Vec3b>(y, center_x) = Vec3b(0, 255, 255); // 使用黄色
            }
            else if (hasLeft && !hasRight) {
                center_x = (leftBoundary + grayImage.cols - 2) / 2;
                // 将中线点画出来
                colorImage.at<Vec3b>(y, center_x) = Vec3b(0, 255, 255); // 使用黄色
            }
            else {
                // 两边都不存在，补线
                center_x = (grayImage.cols - 1) / 2;
                colorImage.at<Vec3b>(y, center_x) = Vec3b(0, 255, 255); // 使用黄色
            }

            // 扩展中线
            for (int x = center_x, i = x - 1, j = x + 1; i >= center_x - center_p_ && j <= center_x + center_p_; i--, j++) {
                if (i >= 0 && i < colorImage.cols) {
                    colorImage.at<Vec3b>(y, i) = Vec3b(0, 255, 255); // 使用黄色
                }
                if (j >= 0 && j < colorImage.cols) {
                    colorImage.at<Vec3b>(y, j) = Vec3b(0, 255, 255); // 使用黄色
                }
            }
        }

        if (img_debug_output_) {
            printUTF8("left_size：" + to_string(left_points.size()) + " right_size：" + to_string(right_points.size()) + " is_stop：" + to_string(is_stop));
        }
        // 判断这一帧是否结束
        if (left_points.size() <= end_dist_ && right_points.size() <= end_dist_ && is_end) is_stop = true;

        return colorImage;
    }

protected:

    // 用于打印
    void printUTF8(const string& str) {
        wstring_convert<codecvt_utf8<wchar_t>> converter;
        wstring wide = converter.from_bytes(str);
        wcout << wide << endl;
    }

    // 判定是否停止
    bool is_stop = false;
    bool is_turn_left = false;
    bool is_turn_right = false;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    // 配置参数
    int range_y_;
    int center_p_;
    int boundary_check_width_;
    bool img_debug_output_;
    int end_dist_;
    string image_topic_;
    int LowH_;
    int HighH_;
    int LowS_;
    int HighS_;
    int LowV_;
    int HighV_;
    int turn_dist_;
    double turn_time_;

    void readParameters() {
        ifstream file("/home/ucar/Desktop/ucar/src/ang_line_follow/config/image.cfg");
        if (!file.is_open()) {
            ROS_ERROR("无法打开图像处理配置文件");
            return;
        }

        string line;
        while (getline(file, line)) {
            // 跳过空行和注释行
            if (line.empty() || line[0] == '#') {
                continue;
            }

            istringstream iss(line);
            string key;
            string value;

            if (getline(iss, key, '=') && getline(iss, value)) {
                // 去除可能的尾随注释
                size_t commentPos = value.find('#');
                if (commentPos != string::npos) {
                    value = value.substr(0, commentPos);
                }

                // 去除首尾空白
                key.erase(0, key.find_first_not_of(" \t"));
                key.erase(key.find_last_not_of(" \t") + 1);
                value.erase(0, value.find_first_not_of(" \t"));
                value.erase(value.find_last_not_of(" \t") + 1);

                // 参数赋值
                try {
                    if (key == "range_y") range_y_ = stoi(value);
                    else if (key == "center_p") center_p_ = stoi(value);
                    else if (key == "boundary_check_width") boundary_check_width_ = stoi(value);
                    else if (key == "img_debug_output") img_debug_output_ = (value == "true" || value == "1");
                    else if (key == "image_topic") image_topic_ = value;
                    else if (key == "end_dist") end_dist_ = stoi(value);
                    else if (key == "LowH") LowH_ = stoi(value);
                    else if (key == "HighH") HighH_ = stoi(value);
                    else if (key == "LowS") LowS_ = stoi(value);
                    else if (key == "HighS") HighS_ = stoi(value);
                    else if (key == "LowV") LowV_ = stoi(value);
                    else if (key == "HighV") HighV_ = stoi(value);
                    else if (key == "turn_dist") turn_dist_ = stoi(value);
                    else if (key == "turn_time") turn_time_ = stod(value);
                    else {
                        ROS_WARN_STREAM("未知参数: " << key << " = " << value);
                    }
                }
                catch (const exception& e) {
                    ROS_ERROR_STREAM("解析参数 " << key << " 时出错: " << e.what());
                }
            }
        }
    }
};

// PID速度控制器 巡线类

class LineFollower : public ImageProcessor {
public:
    LineFollower() : ImageProcessor() {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        locale::global(locale("zh_CN.UTF-8"));

        // 读取参数
        readParameters();

        // 初始化自适应PID参数
        adaptive_Kp_ = Kp_;
        adaptive_Ki_ = Ki_;
        adaptive_Kd_ = Kd_;

        // 打印读取的参数
        printParameters();
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) override {
        ImageProcessor::imageCb(msg);
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge异常: %s", e.what());
            return;
        }

        Mat processed_image = processImage(cv_ptr->image);
        followLine(processed_image);
    }

private:
    ros::Publisher cmd_vel_pub_;
    double max_linear_speed_;
    double max_angular_speed_;
    double Kp_, Ki_, Kd_;
    double adaptive_Kp_, adaptive_Ki_, adaptive_Kd_;
    double deadzone_;
    double integral_limit_;  // 积分限幅
    double error_threshold_;  // 积分分离阈值
    double soft_limit_lower_, soft_limit_upper_;  // 软限幅范围
    double filter_coefficient_;  // 输入滤波系数
    bool pid_debug_output_;
    string end_audio_msg_;      // 结束播报

    // 读取参数配置
    void readParameters() {
        ifstream file("/home/ucar/Desktop/ucar/src/ang_line_follow/config/pid.cfg");
        if (!file.is_open()) {
            ROS_ERROR("无法打开PID配置文件");
            return;
        }

        string line;
        while (getline(file, line)) {
            // 跳过空行和注释行
            if (line.empty() || line[0] == '#') {
                continue;
            }

            istringstream iss(line);
            string key;
            string value;

            if (getline(iss, key, '=') && getline(iss, value)) {
                // 去除可能的尾随注释
                size_t commentPos = value.find('#');
                if (commentPos != string::npos) {
                    value = value.substr(0, commentPos);
                }

                // 去除首尾空白
                key.erase(0, key.find_first_not_of(" \t"));
                key.erase(key.find_last_not_of(" \t") + 1);
                value.erase(0, value.find_first_not_of(" \t"));
                value.erase(value.find_last_not_of(" \t") + 1);

                // 参数赋值
                try {
                    if (key == "max_linear_speed") max_linear_speed_ = stod(value);
                    else if (key == "max_angular_speed") max_angular_speed_ = stod(value);
                    else if (key == "Kp") Kp_ = stod(value);
                    else if (key == "Ki") Ki_ = stod(value);
                    else if (key == "Kd") Kd_ = stod(value);
                    else if (key == "adaptive_Kp") adaptive_Kp_ = stod(value);
                    else if (key == "adaptive_Ki") adaptive_Ki_ = stod(value);
                    else if (key == "adaptive_Kd") adaptive_Kd_ = stod(value);
                    else if (key == "deadzone") deadzone_ = stod(value);
                    else if (key == "integral_limit") integral_limit_ = stod(value);
                    else if (key == "error_threshold") error_threshold_ = stod(value);
                    else if (key == "soft_limit_lower") soft_limit_lower_ = stod(value);
                    else if (key == "soft_limit_upper") soft_limit_upper_ = stod(value);
                    else if (key == "filter_coefficient") filter_coefficient_ = stod(value);
                    else if (key == "pid_debug_output") pid_debug_output_ = (value == "true" || value == "1");
                    else if (key == "end_audio_msg") end_audio_msg_= value;
                    else {
                        ROS_WARN_STREAM("未知参数: " << key << " = " << value);
                    }
                } 
                catch (const exception& e) {
                    ROS_ERROR_STREAM("解析参数 " << key << " 时出错: " << e.what());
                }
            }
        }
      }

    void printParameters() {
        
        printUTF8("============================配置参数相关信息============================");
        printUTF8("最大线速度: " + to_string(max_linear_speed_) + " m/s");
        printUTF8("最大角速度: " + to_string(max_angular_speed_) + " rad/s");
        printUTF8("Kp: " + to_string(Kp_));
        printUTF8("Ki: " + to_string(Ki_));
        printUTF8("Kd: " + to_string(Kd_));
        printUTF8("死区: " + to_string(deadzone_) + " 像素");
        printUTF8("积分限幅: " + to_string(integral_limit_));
        printUTF8("积分分离阈值: " + to_string(error_threshold_));
        printUTF8("软限幅下限: " + to_string(soft_limit_lower_));
        printUTF8("软限幅上限: " + to_string(soft_limit_upper_));
        printUTF8("输入滤波系数: " + to_string(filter_coefficient_));
        printUTF8("PID调试输出: " + string(pid_debug_output_ ? "开启" : "关闭"));
        printUTF8("============================图像配置参数相关信息============================");
        printUTF8("HSV配置: (" + to_string(LowH_) + ", " + to_string(LowS_) + ", " + to_string(LowV_) + "), (" + to_string(HighH_) + ", " + to_string(HighS_) + ", " + to_string(HighV_) + ")");
        printUTF8("图像垂直分割比例: 1/" + to_string(range_y_));
        printUTF8("中心线扩展参数: " + to_string(center_p_));
        printUTF8("边界检测宽度: " + to_string(boundary_check_width_));
        printUTF8("图像调试输出: " + string(img_debug_output_ ? "开启" : "关闭"));
        printUTF8("订阅的图像话题名称: " + image_topic_);
        printUTF8("终点判停阈值: " + to_string(end_dist_));
        printUTF8("转弯判停阈值: " + to_string(turn_dist_));
        printUTF8("转弯所用时间: " + to_string(turn_time_));
        printUTF8("==========================================================================");
    }

    void followLine(const Mat& processedImage) {
        int height = processedImage.rows;
        int width = processedImage.cols;
        
        // 我们将关注图像的底部1/3部分
        int startRow = 2 * height / 3;
        
        vector<Point> yellowPoints;
        
        // 收集底部区域的黄色点
        for (int y = startRow; y < height; y++) {
            for (int x = 0; x < width; x++) {
                Vec3b pixel = processedImage.at<Vec3b>(y, x);
                // 检查像素是否为黄色 (BGR: 0, 255, 255)
                if (pixel[0] == 0 && pixel[1] == 255 && pixel[2] == 255) {
                    yellowPoints.push_back(Point(x, y));
                }
            }
        }
        
        // 判定停止
        if (is_stop) stop();

        if (is_turn_left) {
            rotate_and_move(90.0, turn_time_);  // 90度逆时针旋转
            return;
        }
        if (is_turn_right) {
            rotate_and_move(-90.0, turn_time_);  // 90度顺时针旋转
            return;
        }

        // 计算黄色点的平均x坐标
        int sumX = 0;
        for (const auto& point : yellowPoints) {
            sumX += point.x;
        }
        // 如果没有检测到黄色点，假设线在图像中心
        int avgX = yellowPoints.empty() ? width / 2 : sumX / yellowPoints.size();
        
        // 确定图像中心
        int centerX = width / 2;
        
        // 计算误差（与中心的距离）
        int error = avgX - centerX;
        
        static int lastError = 0;
        static double integral = 0;
        static double lastFilteredError = 0;
        
        // 输入滤波
        double filteredError = filter_coefficient_ * error + (1 - filter_coefficient_) * lastFilteredError;
        lastFilteredError = filteredError;
        
        // 应用死区
        if (abs(filteredError) < deadzone_) {
            filteredError = 0;
        } else {
            filteredError = (filteredError > 0) ? filteredError - deadzone_ : filteredError + deadzone_;
        }
        
        // 自适应PID参数调整
        adaptPIDParameters(filteredError);
        
        // 积分分离
        if (abs(filteredError) < error_threshold_) {
            integral += filteredError;
        } else {
            integral = 0;
        }
        
        // 积分限幅
        integral = max(-integral_limit_, min(integral_limit_, integral));
        
        // 微分先行
        int derivative = filteredError - lastError;
        
        double steeringAngle = adaptive_Kp_ * filteredError + adaptive_Ki_ * integral + adaptive_Kd_ * derivative;
        
        // 软限幅
        steeringAngle = max(soft_limit_lower_, min(soft_limit_upper_, steeringAngle));
        
        geometry_msgs::Twist twist;
        twist.linear.x = max_linear_speed_;
        twist.angular.z = -steeringAngle;
        
        twist.angular.z = max(-max_angular_speed_, min(max_angular_speed_, twist.angular.z));
        
        cmd_vel_pub_.publish(twist);
        if (pid_debug_output_) {
        printSpeedInfo(twist.linear.x, twist.angular.z);
        }
        
        lastError = filteredError;
    }

    // 转弯逻辑
    void rotate_and_move(double target_angle_deg, double rotation_time) {
        // 定义速度消息包
        geometry_msgs::Twist twist;
        // 设置旋转角度，将目标旋转角度转换为弧度
        double target_angle_rad = target_angle_deg * 3.14159 / 180.0;
        // 计算旋转速度
        double angular_speed = target_angle_rad / rotation_time;

        // 设置机器人的旋转速度
        twist.angular.z = angular_speed;

        // 计算旋转的起始时间和发布频率
        double t0 = ros::Time::now().toSec();
        ros::Rate rate(10);

        // 循环发布旋转命令，直到达到旋转时间
        while (ros::Time::now().toSec() - t0 < rotation_time) {
            cmd_vel_pub_.publish(twist);
            rate.sleep();
        }

        // 停止旋转
        twist.angular.z = 0;
        cmd_vel_pub_.publish(twist);
        is_turn_left = false;
        is_turn_right = false;
    }

    void adaptPIDParameters(double error) {
        // 简单的自适应策略示例
        if (abs(error) > error_threshold_ * 2) {
            adaptive_Kp_ = Kp_ * 1.5;
            adaptive_Ki_ = Ki_ * 0.5;
            adaptive_Kd_ = Kd_ * 2;
        } else {
            adaptive_Kp_ = Kp_;
            adaptive_Ki_ = Ki_;
            adaptive_Kd_ = Kd_;
        }
    }

    void stop() {
        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.angular.z = 0;
        cmd_vel_pub_.publish(twist);
        if (pid_debug_output_) printUTF8("停止");
        if (system(("mpg123 " + end_audio_msg_).c_str()) == -1) {
        ROS_WARN("Failed to execute audio command");
        }
        exit(0);
    }

    void printSpeedInfo(double linear_speed, double angular_speed) {
        string direction = (abs(angular_speed) < 0.01) ? "直行" : (angular_speed > 0 ? "左转" : "右转");
        ostringstream oss;
        oss << "线速度: " << linear_speed << " m/s, 角速度: " << angular_speed 
            << " rad/s, 检测到: " << direction;
        printUTF8(oss.str());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ang_line_follow");
    LineFollower lf;
    ros::spin();
    return 0;
}
