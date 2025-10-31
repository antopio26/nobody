## Interface to ROBOT form Jetson

eth1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.123.18  netmask 255.255.255.0  broadcast 192.168.123.255
        inet6 fe80::e619:d1c8:46bd:fcc4  prefixlen 64  scopeid 0x20<link>
        ether 3c:6d:66:1f:77:09  txqueuelen 1000  (Ethernet)
        RX packets 269955  bytes 180803954 (180.8 MB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 1915  bytes 393478 (393.4 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

Robot IP: 192.168.123.20

## Run

ROBOT_IP=192.168.123.20 CONN_TYPE=cyclonedds docker compose up

## Setup DDS

1. run ifconfig
2. search for the interface with ip 192.168.123.18 (eg. eth1)
3. open cyclonedds.xml
4. modify this line with the correct interface name <NetworkInterface name="eth1" priority="default" multicast="default" />


# TODO

Follow this setup:

https://github.com/Unitree-Go2-Robot/go2_robot/tree/humble