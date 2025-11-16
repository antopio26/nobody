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

## References

- [Following Server Documentation](https://docs.nav2.org/configuration/packages/configuring-following-server.html)
- [Dynamic Object Following Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_dynamic_point_following.html)
- [MPPI Controller](https://docs.nav2.org/configuration/packages/configuring-mppic.html)
