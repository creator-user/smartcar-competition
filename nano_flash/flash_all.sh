#!/bin/bash
## config
date_0=$(date)
L4T=/home/robot/Desktop/nano_flash/Linux_for_Tegra-1.6.0.33/Linux_for_Tegra
ROOTFS=/home/robot/Desktop/nano_flash/ucar-mini2-4.2.2-D1.3.8-20220111
PASSWD=123
PTUUID=391bca86-afd0-11ec-93f2-8c8caa798b68
./tf_check.py
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

## check nano dev
echo "check nano device with lsusb"
usb_dev=$(lsusb | grep '0955:7f21 NVidia Corp')
str_len=${#usb_dev}
if [ "$str_len" = '0' ];
then
    echo "NO '0955:7f21 NVidia Corp' usb device. "
    echo "没有发现recovery模式的晓mini，请确认晓mini已经连接且在RECOVERY模式。"
    exit 0
else
    echo ${usb_dev}
fi

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

## flash EMMC
echo "writing partuuid to l4t..."
cd ${L4T}
# sudo ln -s ${ROOTFS} ./rootfs
#uuid=`sudo blkid ${SDCARD}1 | awk '{print $5}' | awk -F'=' '{print $2}'  | sed 's/\"//g'`
uuid=${PTUUID}
echo ${uuid} > bootloader/l4t-rootfs-uuid.txt
echo ${uuid} > bootloader/l4t-rootfs-uuid.txt_ext
echo "flash emmc..."
cd ${L4T}
emmc_result=$(sudo ./flash.sh jetson-nano-emmc external)

if_error=$(echo $emmc_result | grep "Error")
if [[ "$if_error" != "" ]]
then
    echo "Error"
    echo $emmc_result
    echo "EMMC烧录异常，查看上方log。"
    exit 0
else
    echo $emmc_result
fi
if_failed=$(echo $emmc_result | grep "failed")
if [[ "$if_failed" != "" ]]
then
    echo "failed"
    echo $emmc_result
    echo "EMMC烧录异常，查看上方log。"
    exit 0
else
    echo $emmc_result
fi
echo "finish flash emmc..."

## flash SDCARD
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




echo "finish ALL flash. ========================================================================="
echo "finish ALL flash. ==============================烧录完成===================================="
echo "finish ALL flash. ========================================================================="
echo '开始时间：'$date_0
echo '结束时间：'$(date)
