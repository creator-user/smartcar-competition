#!/bin/bash
## config
date_0=$(date)
L4T=/home/robot/Desktop/nano_flash/Linux_for_Tegra-1.6.0.33/Linux_for_Tegra
PASSWD=123
PTUUID=391bca86-afd0-11ec-93f2-8c8caa798b68

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


## flash EMMC
cd ${L4T}

echo "writing partuuid to l4t..."
# sudo ln -s ${ROOTFS} ./rootfs
# uuid=`sudo blkid ${SDCARD}1 | awk '{print $5}' | awk -F'=' '{print $2}'  | sed 's/\"//g'`
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




echo "finish EMMC flash. ========================================================================="
echo "finish EMMC flash. ==============================烧录完成===================================="
echo "finish EMMC flash. ========================================================================="
echo '开始时间：'$date_0
echo '结束时间：'$(date)
