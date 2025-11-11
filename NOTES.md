## Interface to ROBOT form Jetson

``` bash
eth1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.123.18  netmask 255.255.255.0  broadcast 192.168.123.255
        inet6 fe80::e619:d1c8:46bd:fcc4  prefixlen 64  scopeid 0x20<link>
        ether 3c:6d:66:1f:77:09  txqueuelen 1000  (Ethernet)
        RX packets 269955  bytes 180803954 (180.8 MB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 1915  bytes 393478 (393.4 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

Robot IP: 192.168.123.20
```


## Run

On the unitree_go2
```bash
docker compose up unitree_ros
```

On another pc in the network that serves as bridge (better if it is the same running foxglove studio)
```bash
docker compose up foxglove_bridge
```

## Setup DDS

1. run ifconfig
2. search for the interface with ip `192.168.123.18` (eg. `eth1`)
3. open `cyclonedds.xml`
4. modify this line with the correct interface name `<NetworkInterface name="eth1" priority="default" multicast="default" />`

> **NOTE**: With the new implementation this is not necessary anymore.

## Fixes

The pointcloud to laserscan in the go2_driver.launch.py must be modified to be like this:

```python
pointclod_to_laserscan_cmd = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        namespace='',
        output='screen',
        remappings=[('/cloud_in', '/pointcloud')],
        parameters=[{
                'target_frame': 'base_link',
                'max_height': 0.5,
                # 'transform_tolerance': 0.01,
        }],
)
```

## Observations

The voxel map in the other repository is produced by unitree drivers on lower levels. This needs to be investigated further to obtain access to this information.
