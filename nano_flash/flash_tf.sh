#!/bin/bash
## config
date_0=$(date)
ROOTFS=/home/robot/Desktop/nano_flash/ucar-mini2-4.2.2-D1.3.8-20220111
PASSWD=123
PTUUID=391bca86-afd0-11ec-93f2-8c8caa798b68
./tf.py
source ./flash_config

## check SDCARD
if [  "${SDCARD}" = '' ];
then
    echo "NO sdcard exit."
    echo "没有发现TF卡，请确认已经插入TF卡。"
    exit 0
else
    # setup sdcard
    echo "setup_sdcard..."
fi
echo "making APP partition..."

## format SDCARD
sudo umount ${SDCARD}1
sudo umount ${SDCARD}2
echo "d
1
d
w" | sudo fdisk ${SDCARD}

sudo parted ${SDCARD} mklabel gpt yes
sudo parted ${SDCARD} mkpart APP 0GB 50GB # make APP partition
sleep 1
echo y | sudo mkfs.ext4 ${SDCARD}1
sudo parted ${SDCARD} mkpart WORK 50GB 64GB  # make WORK partition
sleep 1
echo y | sudo mkfs.ext4 ${SDCARD}2
sleep 1

## change partuuid ${SDCARD}1
echo "x
c
1
${PTUUID}
m
w
y" | sudo gdisk ${SDCARD}

## change uuid ${SDCARD}1
# sudo tune2fs -U $UUID ${SDCARD}1

## flash SDCARD
###########
echo "flash_sdcard..."

echo "copying rootfs partition..."

cd ${ROOTFS}
sudo mount ${SDCARD}1 /mnt
sudo rm -rf /mnt/*
# sudo tar -cvpf - * | (sudo tar -xvpf - -C /mnt)
sudo cp -rfp ./* /mnt/

echo "syncing rootfs partition..."
echo ${PASSWD}|sudo -S sync
sudo umount /mnt




echo "finish TF flash. ========================================================================="
echo "finish TF flash. ==============================烧录完成===================================="
echo "finish TF flash. ========================================================================="
echo '开始时间：'$date_0
echo '结束时间：'$(date)
