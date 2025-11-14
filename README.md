
Set the lidar IP to `src/nobody_bringup/config/hesai_params.yaml`
```yaml
lidar:
- driver:
    udp_port: 2368                                       #UDP port of lidar
    ptc_port: 9347                                       #PTC port of lidar
    device_ip_address: 192.168.123.20                    #IP address of lidar
    pcap_path: "<Your PCAP file path>"                   #The path of pcap file (set during offline playback)
    correction_file_path: "<Your correction file path>"  #LiDAR angle file, required for offline playback of pcap/packet rosbag
    firetimes_path: "<Your firetime file path>"          #The path of firetimes file
    source_type: 2                                       #The type of data source, 1: real-time lidar connection, 2: pcap, 3: packet rosbag
    pcap_play_synchronization: true                      #Pcap play rate synchronize with the host time
    x: 0                                                 #Calibration parameter
    y: 0                                                 #Calibration parameter
    z: 0                                                 #Calibration parameter
    roll: 0                                              #Calibration parameter
    pitch: 0                                             #Calibration parameter
    yaw: 0                                               #Calibration parameter
ros:
    ros_frame_id: hesai_lidar                            #Frame id of packet message and point cloud message
    ros_recv_packet_topic: /lidar_packets                #Topic used to receive lidar packets from ROS
    ros_send_packet_topic: /lidar_packets                #Topic used to send lidar packets through ROS
    ros_send_point_cloud_topic: /lidar_points            #Topic used to send point cloud through ROS
    send_packet_ros: true                                #true: Send packets through ROS 
    send_point_cloud_ros: true                           #true: Send point cloud through ROS 

    ...

    use_timestamp_type: 1
```


## RUN

``` bash
# Build the image (only needed once or when dependencies change)
docker compose build

# Run both services
docker compose up

# Or run individually
docker compose up unitree_ros
docker compose up foxglove_bridge
```