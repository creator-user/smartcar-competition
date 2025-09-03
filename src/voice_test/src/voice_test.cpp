#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

int awake_flag = 0;    //唤醒标志位


/**************************************************************************
函数功能：唤醒标志sub回调函数
入口参数：唤醒标志位awake_flag_msg  voice_control.cpp
返回  值：无
**************************************************************************/
void awake_flag_Callback(std_msgs::Int8 msg)
{
	awake_flag = msg.data;
	//printf("awake_flag=%d\n",awake_flag);
	
}


/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "voice_test");    //初始化ROS节点
    ros::NodeHandle nh;    //创建句柄

    /***唤醒标志位话题订阅者创建***/
    ros::Subscriber awake_flag_sub = nh.subscribe("awake_flag", 1, awake_flag_Callback);

    /***唤醒标志位话题发布者创建***/
    ros::Publisher awake_flag_pub = nh.advertise<std_msgs::Int8>("awake_flag", 1);
   
    /***任务标志位话题发布者创建***/
    ros::Publisher task_flag_pub = nh.advertise<std_msgs::Int8>("task_flag", 1);

    ros::Rate loop_rate(10);    //循环频率10Hz
    while(ros::ok()){
        if(awake_flag)    //判断休眠状态还是唤醒状态
	    {
            // FILE *fp = NULL;
            // fp = fopen("/home/ucar/Desktop/ucar/src/fame_data.txt", "w+");
            // fputs("小车出发\n", fp);
            // fclose(fp);
            sleep(1);
            system("mpg123 /home/ucar/Desktop/ucar/src/mp3/go.mp3");
            std_msgs::Int8 awake_flag_msg;
            awake_flag = 0;
            awake_flag_msg.data = 0;
            awake_flag_pub.publish(awake_flag_msg);
            std_msgs::Int8 task_flag_msg;
            task_flag_msg.data = 1;  // 小车运动标志位
            task_flag_pub.publish(task_flag_msg);
        }
        ros::spinOnce();    
	loop_rate.sleep();    //10Hz循环
    }
}
