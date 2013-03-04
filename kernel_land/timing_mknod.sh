#!/bin/sh

# Script to mknod for a char device driver after
#      dynamically allocating a Major number

# Written by Scott Brookes with close reference to 
#      O'Rielly's Device Driver text, p. 47

module="timing"
device="timing"

# run insmod
sudo /sbin/insmod ./$module.ko

# remove stale nodes
sudo rm -f /dev/${device}*

# must find dynamically assigned Major number
major=$(cat /proc/devices | grep -m 1 "timing" | cut -d ' ' -f 1)

# make nodes
i=0
while [ $i -lt 13 ]
do
    sudo mknod -m 666 /dev/${device}$i c $major $i
    i=$(($i + 1))
done
