#! /bin/bash

clear
echo userspace > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq > /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq > /sys/devices/system/cpu/cpu0/cpufreq/scaling_setspeed
echo 1 > /sys/devices/system/cpu/cpu0/online

echo userspace > /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor
cat /sys/devices/system/cpu/cpu1/cpufreq/scaling_max_freq > /sys/devices/system/cpu/cpu1/cpufreq/scaling_min_freq
cat /sys/devices/system/cpu/cpu1/cpufreq/scaling_max_freq > /sys/devices/system/cpu/cpu1/cpufreq/scaling_setspeed
echo 1 > /sys/devices/system/cpu/cpu1/online

echo userspace > /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor
cat /sys/devices/system/cpu/cpu2/cpufreq/scaling_max_freq > /sys/devices/system/cpu/cpu2/cpufreq/scaling_min_freq
cat /sys/devices/system/cpu/cpu2/cpufreq/scaling_max_freq > /sys/devices/system/cpu/cpu2/cpufreq/scaling_setspeed
echo 1 > /sys/devices/system/cpu/cpu2/online

echo userspace > /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor
cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_max_freq > /sys/devices/system/cpu/cpu3/cpufreq/scaling_min_freq
cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_max_freq > /sys/devices/system/cpu/cpu3/cpufreq/scaling_setspeed
echo 1 > /sys/devices/system/cpu/cpu3/online

