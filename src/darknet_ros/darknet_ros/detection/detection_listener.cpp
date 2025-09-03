#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/Int8.h>



const char* targetclass=NULL;
std_msgs::Int8 wheel_stop_msg;
// 全局变量，用于存储检测到的目标框
darknet_ros_msgs::BoundingBox targetBox;
int task_flag = 0;
int count = 0;
int flag_det = 0;

ros::Publisher task_flag_pub;
ros::Publisher wheel_stop_pub;

// 发布器，用于发布小车的转向命令
ros::Publisher cmdVelPub;

// 相机图像宽度
const int imageWidth = 640;
const int imageHeight = 480;

const double PI=3.1415926535;

// 转向命令的最大值
const double maxAngularSpeed = 1.0;
const double maxLinearSpeed = 1.0;
// 检测到目标框的回调函数
void boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) {
    if (msg->bounding_boxes.empty() || task_flag != 3) {
        ROS_INFO("No objects detected.");
        // 如果没有找到指定类别的目标框，则将目标框清空
        targetBox = darknet_ros_msgs::BoundingBox();
        return;
    }

    // 遍历所有检测框，找到指定类别的目标框
    for (const auto& box : msg->bounding_boxes) 
    {  
        //ROS_INFO("xmin:%d\nymin:%d\nxmax:%d\nymax:%d\n",box.xmin,box.ymin,box.xmax,box.ymax);
        ROS_INFO("Class:%s",box.Class.c_str());
        if (box.Class == targetclass) {
            flag_det = 1;
            targetBox = box;
            wheel_stop_msg.data = 1;
            wheel_stop_pub.publish(wheel_stop_msg);
            return;
        }
    }
}

void person_num_Callback(std_msgs::Int8 msg)
{
	if(msg.data == 1){
        targetclass = "rod";
    }else if(msg.data == 2){
        targetclass = "bullet_proof";
    }else if(msg.data == 3){
        targetclass = "smoke";
    }
	ROS_INFO("targetclass=%s\n",targetclass);
}

// 发布小车转向命令的函数
void publishCommand(double angularSpeed, double offset_linear) {
    geometry_msgs::Twist cmdVel;
    cmdVel.angular.z = angularSpeed*2.2;
    if(offset_linear < 0.75){
        cmdVel.linear.x = 0.0;
        cmdVel.angular.z = 0.0;
        if(count >= 10 && flag_det == 1){
            count = 0;
            flag_det = 0;
            std_msgs::Int8 task_flag_msg;
            task_flag_msg.data = 1;
            task_flag_pub.publish(task_flag_msg);
            FILE *fp = NULL;
            fp = fopen("/home/ucar/Desktop/ucar/src/fame_data.txt", "w+");
            if(targetclass == "smoke"){
                fputs("我已取到催泪瓦斯", fp);
            }else if((targetclass == "bullet_proof")){
                fputs("我已取到防弹衣", fp);
            }else if((targetclass == "rod")){
                fputs("我已取到警棍", fp);
            }
            fclose(fp);
            system("~/Desktop/ucar/src/Aisound/bin/result.sh");
        }
    }
    else cmdVel.linear.x = 0.38*maxLinearSpeed*offset_linear;
    cmdVelPub.publish(cmdVel);
    //ROS_INFO("%.2lf",offset_linear);
}


void task_flag_Callback(std_msgs::Int8 msg)
{
	task_flag = msg.data;
	ROS_INFO("task_flag=%d\n",task_flag);
}

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "object_tracker");
    ros::NodeHandle nh;

    // 订阅检测结果话题
    ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes",10, boundingBoxesCallback);
    ros::Subscriber person_num_sub = nh.subscribe("person_num", 1, person_num_Callback);
    task_flag_pub = nh.advertise<std_msgs::Int8>("task_flag", 1);
    wheel_stop_pub = nh.advertise<std_msgs::Int8>("wheel_stop", 1);
    ros::Subscriber task_flag_sub = nh.subscribe("task_flag", 1, task_flag_Callback);
    // 初始化转向命令发布器
    cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Rate rate(10); // 设置循环频率为10Hz

    while (ros::ok()) {
        // 如果检测到目标框
        if (!targetBox.Class.empty()) 
        {
            count = 0;
            // 计算目标框在图像中心的偏移量
            double offset_angular = static_cast<double>(targetBox.xmin + targetBox.xmax - imageWidth) / (2.0 * imageWidth);
            
            double hf_x_dis=(targetBox.xmax-targetBox.xmin)/2.0;
            
            double hf_y_dis=(targetBox.ymax-targetBox.ymin)/2.0;
        
            double radius_obj=sqrt(pow(hf_x_dis,2)+pow(hf_y_dis,2));
            
            double radius_cam=sqrt(pow(imageWidth/2.0,2)+pow(imageHeight/2.0,2));
            
            double square_object=PI*radius_obj*radius_obj;
         
            
            double square_camera=PI*radius_cam*radius_cam;
            ROS_INFO("square_object:%.2lf",square_object);
            ROS_INFO("square_camera:%.2lf",square_camera);
              double offset_linear = 1-((float)square_object/square_camera);
            
            ROS_INFO("offset_linear:%.2lf",offset_linear);

            // 根据偏移量计算转向命令
            double angularSpeed = -offset_angular * maxAngularSpeed;
           // double linearSpeed = offset_linear * maxLinearSpeed;
            // 发布转向命令
            publishCommand(angularSpeed, offset_linear);
        } else {
            // 如果没有检测到目标框，则停止转向
            if(flag_det == 1){
                publishCommand(0.0, 0.0);
                count++;
            }
            // ROS_INFO("Stop!");
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}