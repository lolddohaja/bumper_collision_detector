# bumper_collision_detector

```bash
sudo apt install python3-serial
```

```bash
colcon build --packages-select bumper_collision_detector
```

```bash
ros2 launch bumper_collision_detector bumper_monitor.launch.py
```
## Bumper Collision Detector Node Parameters

The Bumper Collision Detector Node utilizes several parameters to customize its behavior according to the specific needs of the application. Below is a description of these parameters:

### `collision_value`

- **Type:** Integer
- **Default:** 100
- **Description:** This parameter sets the threshold for collision detection. A sensor reading above this value indicates the start of a collision event. It is used to filter out minor fluctuations in sensor readings and ensure that only significant changes are considered as collisions.

### `end_collision_value`

- **Type:** Integer
- **Default:** -80
- **Description:** This parameter sets the threshold for identifying the potential end of a collision event. It is used in conjunction with `end_positive_count` to determine when a collision has truly ended. Specifically, it marks the minimum sensor value below which the node starts counting consecutive positive readings to confirm the collision has ended.

### `end_positive_count`

- **Type:** Integer
- **Default:** 10
- **Description:** This parameter specifies the number of consecutive positive sensor readings required to conclude that a collision event has ended. This count starts only after the sensor value has dropped below `end_collision_value`. This parameter helps to prevent false positives in collision end detection by requiring consistent evidence of recovery from a collision.

These parameters allow for fine-tuning of the collision detection logic, enabling it to adapt to different sensor characteristics and application requirements.
