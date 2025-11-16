# NAV2 + Following Server Advanced Usage Guide

This guide explains how to operate both the standard Nav2 point-to-point navigation stack and the dynamic object following capability with the Nav2 Following Server—supported in ROS 2 Humble+ as part of Navigation2.

---

## Overview

- **Classic Navigation:** Use `/navigate_to_pose` action for regular goal-based point-to-point driving.
- **Following Server:** Use `/follow_object` action to follow a moving target (object, person, or arbitrary frame) with smooth tracking.

**Important:** Only send one type of goal at a time—always cancel the current goal before starting a new mode.


## Action Interfaces and Examples

### Classic Navigation

- **Action:** `/navigate_to_pose` ([nav2_msgs/action/NavigateToPose](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action))
- **CLI Example:**
    ```
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 4.0, z: 0.0}, orientation: {w: 1.0}}}}"
    ```
- Used by RViz “Nav2 Goal” by default.

---

### Following Server

- **Action:** `/follow_object` (full doc: [Nav2 Following Server](https://docs.nav2.org/configuration/packages/configuring-following-server.html))
- **Goal Options:** You must specify either a `tracked_frame` (TF frame) or a `pose_topic` (topic publishing geometry_msgs/PoseStamped).

#### A. Follow by TF Frame

You have a dynamic object (target) publishing a TF frame (e.g., detected by vision/perception):

```
ros2 action send_goal /follow_object nav2_msgs/action/FollowObject "{tracked_frame: 'person_frame', max_duration: 0.0}"
```

- Replace `'person_frame'` with the dynamic frame you want to follow (e.g., from a leg/person tracker).
- `max_duration: 0.0` means follow indefinitely.

#### B. Follow by Pose Topic

You have a detection system publishing `geometry_msgs/PoseStamped` to a topic (say `/detected_object_pose`):

```
ros2 action send_goal /follow_object nav2_msgs/action/FollowObject "{pose_topic: '/detected_object_pose', max_duration: 0.0}"
```

- The pose on this topic will be followed in real time.

---

## Behavior Trees (BT) Integration

To automate switching between navigation and following, or combine with other robot actions, use a Behavior Tree (BT).

**Example BT XML:**
```
<BehaviorTree ID="MainTree">
    <Fallback name="choose_behavior">
        <SequenceStar>
            <CheckIfFollowingRequested/>
            <FollowObject tracked_frame="person_frame" max_duration="0.0"/>
        </SequenceStar>
        <SequenceStar>
            <NavigateToPose/>
        </SequenceStar>
    </Fallback>
</BehaviorTree>
```

- BT allows flexible autonomy logic (manual goal or auto-follow based on external commands).
- See [BT Navigator docs](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html) for custom nodes and monitoring.

## Mode Switching and Conflict Avoidance

- **Always cancel previous action** before starting a new one:
  ```
  ros2 action cancel /navigate_to_pose
  ros2 action cancel /follow_object
  ```
- Only *one* server should issue velocity commands at a time (both use `/cmd_vel`).


## Monitoring & Diagnostics

- **Status of navigation:** `/navigate_to_pose/_action/status`
- **Status of following:** `/follow_object/_action/status`
- **Object detection:** Custom, e.g. `/detected_object_pose` (for pose topic) or TF monitor (for frames).

---

## Troubleshooting

- Make sure object pose/TF is updated at a reasonable rate (~10Hz+).
- If the robot “wanders” after target disappears, check detection timeouts.
- Only one thing should publish to `/cmd_vel` at a time.
- Check remappings if using namespaces or multi-robot setups.

---

## FAQ

- **What does the following_server need?**  
  The robot base must subscribe to `/cmd_vel`, and either a dynamic TF frame or a topic with a high-frequency pose for the target.
- **Can I follow people?**  
  Yes! Use a people-tracking pipeline that publishes a `geometry_msgs/PoseStamped` or TF.
- **How do I test without a real detector?**  
  Use `ros2 topic pub /detected_object_pose geometry_msgs/PoseStamped ...` to simulate a moving target.
- **What happens when the target is lost?**  
  The following_server will stop the robot after `detection_timeout` and can optionally search for the object if so configured.

---

## Spatio-Temporal Voxel Layer (STVL)

This system uses the **Spatio-Temporal Voxel Layer** as a modern replacement for the standard `voxel_layer` in Nav2. STVL provides efficient 3D environmental representation with temporal decay capabilities.

### What is STVL?

STVL is a drop-in replacement for the traditional voxel grid that leverages OpenVDB (a library from Dreamworks) to efficiently store and manipulate sparse 3D volumetric data. It provides:

- **Spatial Representation:** Environment stored as a configurable voxel grid with very low memory overhead
- **Temporal Decay:** Voxels expire over time using configurable decay models
- **Frustum Acceleration:** Intelligent clearing of space within sensor FOV
- **Real-time Performance:** Handles multiple dense sensors (e.g., 7 depth cameras) with minimal CPU usage

### Key Features

1. **Efficient Memory Usage:** A 60,000 sq.ft. retail store with 710,765 voxels at 0.05m resolution uses only 6.45MB
2. **Temporal Intelligence:** Voxels decay over time rather than persisting indefinitely
3. **Frustum-based Clearing:** Accelerates voxel decay within sensor field-of-view without costly raytracing
4. **Dual Sensor Support:** Depth cameras (cubical frustum) and 3D LiDARs (hourglass frustum)
5. **Real-time Filtering:** Built-in voxel and passthrough filters to reduce computational overhead

### Configuration Overview

STVL requires **two observation sources per sensor**: one for marking obstacles and one for clearing free space.

#### Marking Source
Detects and marks obstacles in the environment:
```yaml
hesai_mark:
  data_type: "PointCloud2"
  topic: /lidar_points
  marking: true
  clearing: false
  min_obstacle_height: 0.1
  max_obstacle_height: 0.7
  voxel_filter: "voxel"  # Reduces CPU overhead
```

#### Clearing Source (Frustum Acceleration)
Accelerates decay of voxels within sensor FOV:
```yaml
hesai_clear:
  enabled: true
  data_type: "PointCloud2"
  topic: /lidar_points
  marking: false
  clearing: true
  min_z: 0.1
  max_z: 7.0
  vertical_fov_angle: 0.52    # radians (~30°)
  horizontal_fov_angle: 6.28  # radians (360°)
  decay_acceleration: 5.0     # 1/s^2
  model_type: 1               # 0=depth camera, 1=3D lidar
```

### Decay Models

Control how voxels expire over time using `decay_model`:

- **`-1` (Persistent):** Voxels never decay - permanent mapping
- **`0` (Linear):** Voxels decay linearly over `voxel_decay` seconds
- **`1` (Exponential):** Voxels decay exponentially (e^-t)

**Recommended Settings:**
- **Local Costmap:** `voxel_decay: 15.0`, `decay_model: 0` (linear, 15 seconds)
- **Global Costmap:** `voxel_decay: 30.0`, `decay_model: 0` (linear, 30 seconds for persistent memory)

### Sensor Model Types

Set `model_type` based on your sensor:

- **`0` (Depth Camera):** Traditional 6-plane cubical frustum (e.g., Intel RealSense, Unitree Go2 camera)
- **`1` (3D LiDAR):** Hourglass-shaped FOV (e.g., Velodyne VLP-16, Hesai)

### Real-time Source Toggle

Enable or disable observation sources dynamically:

```bash
# Enable a source
ros2 service call /local_costmap/stvl_layer/hesai_mark/toggle_enabled std_srvs/srv/SetBool "{data: true}"

# Disable a source
ros2 service call /local_costmap/stvl_layer/unitree_clear/toggle_enabled std_srvs/srv/SetBool "{data: false}"
```

### Visualization

STVL publishes a voxel map point cloud for visualization in RViz:
- **Topic:** `/local_costmap/voxel_marked_cloud` and `/global_costmap/voxel_marked_cloud`
- **Enable:** Set `publish_voxel_map: true` in configuration

**Performance Note:** Visualizing very large voxel grids can impact performance. Disable visualization during nominal operations if your decay time is long (>15 seconds).

### Performance Optimization

1. **Voxel Filter:** Use `voxel_filter: "voxel"` on observation sources to reduce point cloud density
2. **Voxel Min Points:** Set `voxel_min_points: 0` to keep all voxels after filtering
3. **Update Rates:** 
   - Local costmap: 15Hz update, 10Hz publish
   - Global costmap: 2Hz update, 1Hz publish
4. **Clear After Reading:** Set `clear_after_reading: true` to prevent stale data accumulation

### Troubleshooting

- **High CPU Usage:** Enable voxel filtering on dense sensor sources
- **Objects Not Clearing:** Check `decay_acceleration` and FOV angles match your sensor specs
- **Memory Issues (Ubuntu 20.04/22.04):** Set `export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libjemalloc.so.2` due to OpenVDB binary issue

### Mapping Mode

STVL can be used for 3D environmental mapping:
```yaml
mapping_mode: true          # Enable mapping (disables navigation features)
map_save_duration: 60.0     # Auto-save interval in seconds
```

Maps are saved in `.vdb` format and can be visualized with `vdb_viewer`.

---

## References

- [Following Server Documentation](https://docs.nav2.org/configuration/packages/configuring-following-server.html)
- [Dynamic Object Following Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_dynamic_point_following.html)
- [MPPI Controller](https://docs.nav2.org/configuration/packages/configuring-mppic.html)
- [STVL GitHub Repository](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/tree/ros2)
- [STVL Research Paper](https://journals.sagepub.com/doi/10.1177/1729881420910530)
