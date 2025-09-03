#!/usr/bin/python3
# -*- coding: UTF8 -*-
import os
passwd="123"
os.popen("echo " +passwd + " | sudo -S  fdisk -l")
dev_sdx = os.popen("echo "+passwd +"|sudo -S fdisk -l |grep 'Disk /dev/sd'").read().split("\n")
dev_name = ''
tf_on=False
for  line in dev_sdx:
    if  '59.2 GiB' in line:
        if  ':' in line:
            dev_name = line.split(':')[0].split(' ')[1]
            os.system("echo SDCARD="+dev_name+" > ./flash_config")
            tf_on=True
            break
        elif  '：' in line:
            dev_name = line.split('：')[0].split(' ')[1]
            os.system("echo SDCARD="+dev_name+" > ./flash_config")
            tf_on=True
            break
if tf_on==False:
    os.system("echo SDCARD="+" > ./flash_config")
    print("no target tf card!")
# dev_mmcx = os.popen("echo '123'|sudo -S fdisk -l |grep 'Disk /dev/mmcblk'").read()