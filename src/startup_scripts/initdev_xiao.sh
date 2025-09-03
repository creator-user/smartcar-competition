#!/bin/bash
# 待修改
echo 'source /home/ucar/ucar_ws/devel/setup.bash' >> /home/ucar/.bashrc # 小车底盘
echo 'KERNEL=="ttyUSB*", SUBSYSTEMS=="usb",ATTRS{idVendor}=="10c4",ATTRS{idProduct}=="ea60",KERNELS=="1-2.4",NAME="ttyUSB0",SYMLINK+="BASE_Serial_port"' >  /etc/udev/rules.d/ucar.rules
echo 'KERNEL=="video*", SUBSYSTEMS=="usb",ATTRS{idVendor}=="0edc",ATTRS{idProduct}=="2050",KERNELS=="1-1",NAME="video0",SYMLINK+="ucar_video"' >>  /etc/udev/rules.d/ucar.rules
echo 'ATTRS{idVendor}=="10d6", ATTRS{idProduct}=="b003", MODE="0666"' >>  /etc/udev/rules.d/ucar.rules # 麦克风阵列
echo 'KERNEL=="ttyTHS1" MODE="0666"' >>  /etc/udev/rules.d/ucar.rules
service udev reload
sleep 2
service udev restart
