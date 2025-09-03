# smartcar-competition
全国大学生智能汽车竞赛项目代码文件

## 项目结构
```
smartcar-competition/
├── nano_flash/ # Jetson Nano 刷机与配置脚本
│ ├── flash_all.sh
│ ├── flash_config
│ ├── flash_emmc.sh
│ └── flash_tf.sh
├── src/ # ROS 工作空间源码
│ ├── Aisound/ # 讯飞语音合成模块
│ ├── ang_line_follow/ # 角度循迹小车程序
│ ├── darknet_ros/ # 目标检测(ROS封装)
│ ├── fdilink_ahrs/ # 惯性测量单元(IMU/AHRS) 驱动
│ ├── geometry/ # ROS 几何库
│ ├── geometry2/ # ROS tf2库
│ ├── line_follower/ # 赛道循迹模块
│ ├── line_follower2/ # 第二版循迹模块
│ ├── main/ # 主程序 (启动/导航/语音交互)
│ ├── mp3/ # 语音提示音频文件
│ ├── race_navigation/ # 比赛赛道导航策略
│ ├── startup_scripts/ # 系统初始化脚本
│ ├── teb_local_planner/ # TEB 局部路径规划器
│ ├── tmp/ # 临时运行目录
│ ├── ucar_camera/ # 摄像头驱动与图像采集
│ ├── ucar_controller/ # 小车底盘控制
│ ├── ucar_map/ # 地图构建与管理
│ ├── ucar_nav/ # 导航模块
│ ├── vision/ # 循迹算法
│ ├── vision_opencv/ # ROS的OpenCV接口
│ ├── voice_test/ # 语音交互测试
│ ├── xf_mic_asr_offline/ # 离线语音识别
│ ├── YDLidar-SDK/ # YDLidar 激光雷达SDK
│ └── ydlidar_ros_driver/ # YDLidar 激光雷达ROS驱动
├── 技术报告.docx
└── README.md # 项目说明文档
```
