#!/usr/bin/env bash

#sudo ifconfig lo multicast
#sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
sudo ifconfig wlan0 multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev wlan0

export LCM_DEFAULT_URL=udpm://239.255.255.250:7667?ttl=3

#lcm-logger -s ./log/lcm-log-%F-%T &
#. ./runspy.sh &

#./tenDOF /dev/i2c-1
./gyroscope /dev/i2c-1

kill %1 %2

