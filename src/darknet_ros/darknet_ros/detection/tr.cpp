#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/Int8.h>
#include <mutex>

// 全局变量，用于存储目标类别和检测到的目标框
std::string target_class = "";
std_msgs::Int8 wheel_stop_msg;
darknet_ros_msgs::BoundingBox target_box;
int task_flag = 0;
int count = 0;
int flag_det = 0;
double offset_angular;

// 互斥锁，用于保护全局变量的访问
std::mutex mtx;

// 发布器，用于发布停止、测量和转向命令
ros::Publisher wheel_stop_pub;
ros::Publisher measure_pub;
ros::Publisher cmd_vel_pub;

// 相机图像的宽度和高度
const int image_width = 640;
const int image_height = 480;

// 常量，用于设置最大转向速度
const double PI = 3.1415926535;
const double max_angular_speed = 0.5;

// 检测到目标框的回调函数
void boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) {
    if (msg->bounding_boxes.empty() || task_flag != 3 || flag_det != 0) {
        ROS_INFO("No objects detected.");
        // 如果没有找到指定类别的目标框，则将目标框清空
        std::lock_guard<std::mutex> lock(mtx);
        target_box.Class.clear();
        return;
    }

    // 遍历所有检测框，找到指定类别的目标框
    for (const auto& box : msg->bounding_boxes) {
        if (box.Class == target_class) {
            ROS_INFO("Class: %s", box.Class.c_str());
            std::lock_guard<std::mutex> lock(mtx);
            target_box = box;
            wheel_stop_msg.data = 1;
            wheel_stop_pub.publish(wheel_stop_msg);
            return;
        }
    }
}

// 人数信息的回调函数，根据人数设置目标类别
void personNumCallback(const std_msgs::Int8::ConstPtr& msg) {
    if (msg->data == 1) {
        target_class = "rod";
    } else if (msg->data == 2) {
        target_class = "bullet_proof";
    } else if (msg->data == 3) {
        target_class = "smoke";
    }
    ROS_INFO("target_class=%s\n", target_class.c_str());
}

// 发布小车转向命令的函数
void publishCommand(double angular_speed) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.z = angular_speed * 2.2;
    if (abs(angular_speed) <= 0.05 && flag_det != 1) {
        count++;
        cmd_vel.angular.z = 0.0;
        if (count >= 10) {
            count = 0;
            std_msgs::Int8 msg;
            msg.data = 1;
            measure_pub.publish(msg);
            flag_det = 1;
            ROS_INFO("flag_det=%d\n", flag_det);
        }
    } else {
        count = 0;
    }
    cmd_vel_pub.publish(cmd_vel);
}

// 任务标志的回调函数
void taskFlagCallback(const std_msgs::Int8::ConstPtr& msg) {
    task_flag = msg->data;
    ROS_INFO("task_flag=%d\n", task_flag);
}

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "object_tracker");
    ros::NodeHandle nh;

    // 订阅检测结果话题
    ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes", 10, boundingBoxesCallback);
    ros::Subscriber person_num_sub = nh.subscribe("person_num", 1, personNumCallback);
    ros::Subscriber task_flag_sub = nh.subscribe("task_flag", 1, taskFlagCallback);

    // 初始化发布器
    wheel_stop_pub = nh.advertise<std_msgs::Int8>("wheel_stop", 1);
    measure_pub = nh.advertise<std_msgs::Int8>("/start_measurement", 1);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Rate rate(10); // 设置循环频率为10Hz

    while (ros::ok()) {
        // 如果检测到目标框
        if (!target_box.Class.empty() && flag_det == 0) {
            std::lock_guard<std::mutex> lock(mtx);
            // 计算目标框在图像中心的偏移量
            offset_angular = static_cast<double>(target_box.xmin + target_box.xmax - image_width) / (2.0 * image_width);

            // 根据偏移量计算转向命令
            double angular_speed = -offset_angular * max_angular_speed;
            // 发布转向命令
            publishCommand(angular_speed);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
