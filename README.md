# Argo - Nobody

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
