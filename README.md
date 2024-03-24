English| [简体中文](./README_cn.md)

# Function Introduction

The `parking_perception` package is an outdoor parking area detection algorithm example developed based on the `hobot_dnn` package. It utilizes a multi-task reasoning model introduced by Horizon, using the parking area detection model on the Horizon Sunrise X3 platform and indoor data for model reasoning with BPU, thus obtaining AI reasoning results.

This package supports directly subscribing to topics of type `sensors/msg/Image` and supports inferring from local images. The AI information will be published through the topic while the results are visualized on a web page. It also supports saving the rendered images in the "result" directory during program execution.

The multi-task model simultaneously supports semantic segmentation and object detection.
The algorithm supports the following semantic segmentation categories:
```
1. Road
2. Background
3. Lane marking
4. Sign line
5. Parking lane
6. Parking space
7. Parking rod
8. Parking lock
```

The algorithm supports the following object detection categories:
```
1. Cyclist
2. Person
3. Rear
4. Vehicle
5. Parking lock
```

Each category includes information such as length, width, and category. The package externally publishes AI Msg containing semantic segmentation and object detection information, which users can subscribe to for application development.
The complete description of the AI Msg is as follows:

````
# Segmentation Message
Capture[] captures
Data Structure:
std::vector<float> features

# Segmentation type names, e.g., road/background/lane_marking/sign_line/parking_lane/parking_space/parking_rod/parking_lock

# Detection Message
Roi[] rois
Data Structure:
std::string type
int rect.x_offset
int rect.y_offset
int rect.width
int rect.height

# Detection type names, e.g., cyclist/person/rear/vehicle/parkinglock```

# Compilation

## Dependencies

ros package:

- dnn_node
- ai_msgs
- OpenCV

dnn_node is a package for model inference using the BPU processor on the Horizon Sunrise X3 development board, defined in hobot_dnn.

ai_msgs is a custom message format used to publish inference results after algorithm model inference, defined in hobot_msgs.

OpenCV is used for image processing.

## Development Environment

- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0/Linaro GCC 9.3.0

## Compilation

Supports compilation on X3 Ubuntu system and cross-compilation using Docker on PC.

### Compilation on X3 Ubuntu System

1. Compilation Environment Confirmation
   - X3 Ubuntu system is installed on the board.
   - The current compilation terminal has set the TogetherROS environment variable: `source PATH/setup.bash`. Replace PATH with the installation path of TogetherROS.
   - ROS2 compilation tool colcon is installed, installation command: `pip install -U colcon-common-extensions`

2. Compilation
   Compilation command: `colcon build --packages-select parking_perception`

### Cross-Compilation using Docker for X3

1. Compilation Environment Confirmation
   - Compilation is done in Docker, with TogetherROS already installed in Docker. For Docker installation, cross-compilation instructions, TogetherROS compilation, and deployment details, refer to the README.md in the robot development platform robot_dev_config repo.

2. Compilation
```- Compilation command:

```
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select parking_perception \
   --merge-install \
   --cmake-force-configure \
   --cmake-args \
   --no-warn-unused-cli \
   -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
```

### Compilation for X86 version on X86 Ubuntu system

1. Compilation Environment Confirmation

   - X86 Ubuntu version: ubuntu20.04

2. Compilation

   - Compilation command:

   ```
   colcon build --packages-select parking_perception  \
      --merge-install \
      --cmake-args \
      -DPLATFORM_X86=ON \
      -DTHIRD_PARTY=`pwd`/../sysroot_docker \
   ```

## Points to Note

# Introduction

## Dependencies

- parking_perception package: Publishes 2D detection and segmentation information

## Camera Input

- image_width: 640
- image_height: 320

## Parameters

| Parameter Name         | Type        | Description                                  | Required | Supported Configurations | Default Value                 |
| ---------------------- | ----------- | -------------------------------------------- | -------- | ------------------------ | ----------------------------- || ai_msg_pub_topic_name  | std::string | Topic name for publishing AI messages containing detection results | No | Configured based on actual deployment environment | /ai_msg_parking_perception |
| image_sub_topic_name | std::string | Topic name to subscribe to in the ROS2 message list | No | Configured based on actual deployment environment | /image_raw |
| dump_render_img | int | Whether to save rendered images locally | No | Saved in the "result" directory | 0 |
| shared_mem  | int | Whether to use shared memory communication to subscribe to image messages. 0: Off; 1: On. The topic names for subscribing to images using shared memory communication are /hbmem_img and /image_raw for Off and On respectively | No | 0/1 | 1 |

## Running

After successful compilation, copy the generated install directory to the Horizon X3 development board (ignore copying steps if compiling on X3), and run the following commands:

### **Ubuntu X3**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# For the config, copy according to the actual installation path
# If compiling on board (without --merge-install option), use cp -r install/PKG_NAME/lib/PKG_NAME/config/ . for copying, where PKG_NAME is the specific package name.
cp -r install/lib/parking_perception/config/ .

# Launch the parking detection node
ros2 run parking_perception parking_perception

```

### **Ubuntu X3 Launch**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# For the config, copy according to the actual installation path
# If compiling on board (without --merge-install option), use cp -r install/PKG_NAME/lib/PKG_NAME/config/ . for copying, where PKG_NAME is the specific package name.
cp -r install/lib/parking_perception/config/ .

# Launch the parking detection package
# Detection using mipi camera input, render results visualized on a web page and saved locally
export CAM_TYPE=mipi
ros2 launch parking_perception parking_perception.launch.py 

# Render results for a single image, visualized on a web page and saved locally
export CAM_TYPE=fb
ros2 launch parking_perception parking_perception.launch.py 

```

### **Linux X3**

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# For the config, use an example model for copying based on the actual installation path```cp -r install/lib/parking_perception/config/ .

# Start parking detection node
./install/lib/parking_perception/parking_perception

```

### **X86 Ubuntu**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# Copy example models from config based on actual installation path
cp -r install/lib/parking_perception/config/ .

# Start parking detection node
./install/lib/parking_perception/parking_perception

```

# Result Analysis
## X3 Results Display
Original Image & Intelligent Result

<img src="config/images/2.jpg" width="640" height="320"  alt="Original Image"/><br/>

<img src="render.png" width="640" height="320"  alt="Intelligent Result"/><br/>

## Result Explanation
In this example, the results of inference on local images are rendered on a webpage. From the visualization, it can be seen that in the outdoor scene, the parking area and driving area are effectively segmented, distinguishing between parking lane lines and driving lane lines, and the object detection task can locate vehicles in the distance.

When "dump_render_img" is set to 1, the rendered images are saved in real-time in the result directory of the current path.

# Frequently Asked Questions

Q1: After enabling "dump_render_img", the frame rate decreases and sometimes DNN reports an ERROR.

A1: Local rendering and saving consume resources and increase power consumption, leading to CPU throttling and reduced detection speed.