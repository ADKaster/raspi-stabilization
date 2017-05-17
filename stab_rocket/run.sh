#!/usr/bin/env bash

#ifconfig lo multicast
#route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
#ifconfig wlan0 multicast
#route add -net 224.0.0.0 netmask 240.0.0.0 dev wlan0
ifconfig eth0 multicast
route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0


export LCM_DEFAULT_URL=udpm://239.255.255.250:7667?ttl=1

#lcm-logger -s ./log/lcm-log-%F-%T &
#. ./runspy.sh &

#./tenDOF /dev/i2c-1
./gyroscope /dev/i2c-1

kill %1 %2

