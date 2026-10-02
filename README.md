# ROS 2 Camera

A ROS 2 Jazzy C++ package that publishes images from a webcam, reports received
images, and displays the image stream using OpenCV.

## Build

From the workspace root, source ROS 2 Jazzy and build the package:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros_camera --symlink-install
source install/setup.bash
```

If `colcon` is not installed, install it with `sudo apt install python3-colcon-common-extensions`.

## Run the image publisher, subscriber, and viewer

Open three terminals in the workspace. In each terminal, source the ROS and
workspace setup files:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Then run one node in each terminal:

**Terminal 1 — webcam image publisher**

```bash
ros2 run ros_camera image_publisher
```

**Terminal 2 — image subscriber**

```bash
ros2 run ros_camera image_subscriber
```

**Terminal 3 — image viewer**

```bash
ros2 run ros_camera image_viewer
```

The publisher captures from camera index `0` at the camera's default resolution and
publishes `sensor_msgs/msg/Image` messages on `camera/image_raw`. Image dimensions
are taken from each captured frame, so they reflect the camera output and update if
the frame size changes. The subscriber logs image dimensions and encoding;
the viewer opens an OpenCV window for the same topic. The viewer needs access to a
graphical desktop. Stop a node with `Ctrl+C`.

Optional `width` and `height` publisher parameters request a camera resolution; both
default to `0`, which leaves resolution selection to the camera. Camera and stream
settings can be overridden at startup, for example:

```bash
ros2 run ros_camera image_publisher --ros-args -p camera_index:=1 -p width:=1280 -p height:=720 -p publish_rate_hz:=15.0
```

To use a different topic, set `topic_name` to the same value for the publisher,
subscriber, and viewer. For example:

```bash
ros2 run ros_camera image_subscriber --ros-args -p topic_name:=my_camera/image
```
