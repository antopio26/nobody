# Argo - Nobody

![Argo navigation visualized via Foxglove](images/cover.gif)

This repository contains the a ROS2 package and docker setup to run all the code needed to operate the Unitree GO2 EDU robot from an external Foxglove interface.

It includes a basic Nav2 setup that merges the info from the onboard lidar and additional Hesai lidar. Foxglove layouts will be available in the future.

All the configurations are available inside `src/nobody_bringup/config/` folder.

## Run

To run the package, install docker on the extension module and run this commands:

``` bash
# Build the image (only needed once or when dependencies change)
docker compose build

# Run both services
docker compose up

# Or run individually
docker compose up unitree_ros
docker compose up foxglove_bridge
```

## Setup DDS

1. run ifconfig
2. search for the interface with ip `192.168.123.18` (eg. `eth1`)
3. open `cyclonedds.xml`
4. modify this line with the correct interface name `<NetworkInterface name="eth1" priority="default" multicast="default" />`

## Config Hesai Lidar

Set the lidar IP in `src/nobody_bringup/config/hesai_params.yaml` and make sure the timestamp is taken from ros:

```yaml
lidar:
- driver:
    ...
    device_ip_address: 192.168.123.20                    #IP address of lidar
    ...
ros:
    ...
    use_timestamp_type: 1
    ...
```
