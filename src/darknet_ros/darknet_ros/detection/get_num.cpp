#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <std_msgs/Int8.h>


// 全局变量，用于存储检测到的目标框
darknet_ros_msgs::BoundingBox targetBox;

// 全局变量，用于存储任务标志位和人数发布者
ros::Publisher task_flag_pub;
ros::Publisher person_num_pub;

//全局变量，存储识别到的人数
std_msgs::Int8 person_num_msg;
//全局变量，存储任务标志位
int task_flag = 0;

// 检测到目标框的回调函数
void boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) {
    if (msg->bounding_boxes.empty() || task_flag != 2) {
        ROS_INFO("No objects detected.");
        return;
    }
    // 遍历所有检测框，找到指定类别的目标框
    for (const auto& box : msg->bounding_boxes) {
        if (box.Class == "terrorist_one" || box.Class == "terrorist_two" || box.Class == "terrorist_three") {
            targetBox = box;
            return;
        } 
    }

    // 如果没有找到指定类别的目标框，则将目标框清空
    targetBox = darknet_ros_msgs::BoundingBox();
}

void task_flag_Callback(std_msgs::Int8 msg)
{
	task_flag = msg.data;
	ROS_INFO("task_flag=%d\n",task_flag);
}


// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "get_person_num");
    ros::NodeHandle nh;

    // 订阅检测结果话题
    ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes", 10, boundingBoxesCallback);

    /***任务标志位话题发布者创建***/
    task_flag_pub = nh.advertise<std_msgs::Int8>("task_flag", 1);


   /***任务标志位话题订阅者创建***/
    ros::Subscriber task_flag_sub = nh.subscribe("task_flag", 1, task_flag_Callback);

    /**识别人数话题发布者创建***/
    person_num_pub = nh.advertise<std_msgs::Int8>("person_num", 1);
   ros::Rate loop_rate(10);    //循环频率10Hz
    while(ros::ok()){
        // 如果检测到目标框
        if (!targetBox.Class.empty()) {
            if (targetBox.Class == "terrorist_one") {
            // targetBox = box;
            person_num_msg.data = 1;
            person_num_pub.publish(person_num_msg);
            }
            else if (targetBox.Class == "terrorist_two") {
                // targetBox = box;
                person_num_msg.data = 2;
                person_num_pub.publish(person_num_msg);
            }
            else if (targetBox.Class == "terrorist_three") {
                // targetBox = box;
                person_num_msg.data = 3;
                person_num_pub.publish(person_num_msg);
            }
            // 播放识别结果
            if(person_num_msg.data == 1 || person_num_msg.data == 2 || person_num_msg.data == 3){
                // FILE *fp = NULL;
                // fp = fopen("/home/ucar/Desktop/ucar/src/fame_data.txt", "w+");
                // fputs("恐怖分子数量为", fp);
                // fputs(std::to_string(person_num_msg.data).c_str(), fp);
                // fputs("个\n", fp);
                // fclose(fp);
                switch(person_num_msg.data){
                    case 1:system("mpg123 /home/ucar/Desktop/ucar/src/mp3/k1.mp3");
                    break;
                    case 2:system("mpg123 /home/ucar/Desktop/ucar/src/mp3/k2.mp3");
                    break;
                    case 3:system("mpg123 /home/ucar/Desktop/ucar/src/mp3/k3.mp3");
                    break;
                }
                
                std_msgs::Int8 task_flag_msg;
                task_flag_msg.data = 1;
                task_flag_pub.publish(task_flag_msg);
                break;
            }
        }
        ros::spinOnce();    
	    loop_rate.sleep();    //10Hz循环
    }
    return 0;
}
