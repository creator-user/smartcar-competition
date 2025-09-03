# smartcar-competition
全国大学生智能汽车竞赛项目代码文件

## 项目结构
```
smartcar-competition/
├── nano_flash/ # Nano 开发板刷机脚本与配置
│ ├── flash_all.sh
│ └── Linux_for_Tegra-1.6.0.33.tar.xz (已移除 >100MB 文件)
├── src/ # ROS 工作空间源码
│ ├── darknet_ros/ # 目标检测 (YOLO)
│ ├── teb_local_planner/ # 路径规划
│ ├── vision_opencv/ # OpenCV 封装
│ ├── ang_line_follow/ # 赛道循迹模块
│ ├── line_follower/ # 循迹车控制
│ ├── ucar_controller/ # 底盘控制
│ ├── ucar_nav/ # 导航
│ ├── ucar_map/ # 地图
│ ├── race_navigation/ # 竞赛导航策略
│ ├── vision/ # 视觉检测与路径跟随脚本
│ └── YDLidar-SDK/ # 激光雷达驱动
├── scripts/ # 启动与测试脚本
├── docs/ # 文档资料
└── README.md
```
