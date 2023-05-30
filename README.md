# Package Name

## Overview

ROS wrapper for XV-SDK from XVisio Technology.

**Keywords:** slam, visual, camera, VR, AR, XR, inertial, tof, depth, rgb, color, fisheye

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

Author: [XVisio Technology](https://www.xvisiotech.com/)<br />
Maintainer: Ange Nizard, ange@xvisiotech.com

The xv_sdk package has been tested under [ROS] Melodic on Ubuntu 18.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.
It is assumed that the user knows the basics of ROS.

## Installation

### Building from Source

#### Dependencies

- [ROS], to install ROS Melodic on Ubuntu it's [here](http://wiki.ros.org/melodic/Installation/Ubuntu)
- Your XVisio XV-SDK binaries and include files

#### Building

To build from source, copy the latest version of `xv_sdk` package into the `src` sub-folder of your catkin workspace: `catkin_workspace/src/` and then compile the package using

	cd catkin_workspace
    rosdep install --from-paths src --ignore-src -r -y
	catkin_make -DXVSDK_INCLUDE_DIRS="/path/to/xvsdk/include" -DXVSDK_LIBRARIES="/path/to/libxvsdk.so"

If you are using an official release of the XV-SDK library, the include directoy and `libxvsdk.so` are provided.
If you have access to the proprietary source-code of XV-SDK and want to use it, you need to build it to get `libxvsdk.so`

### Running in Docker (optional)

Docker is a great way to run an application with all dependencies and libraries bundles together. 
Make sure to [install Docker](https://docs.docker.com/get-docker/) first. 

First, spin up a simple container:

	docker run -ti --rm --name ros-container ros:melodic bash
	
This downloads the `ros:melodic` image from the Docker Hub, indicates that it requires an interactive terminal (`-t, -i`), gives it a name (`--name`), removes it after you exit the container (`--rm`) and runs a command (`bash`).

Now, create a catkin workspace, clone the package, build it, done!

	apt-get update && apt-get install -y git
	mkdir -p /catkin_workspace/src && cd /catkin_workspace
	cp path/to/package ./src/
    rosdep install --from-path src -r -y
	catkin_make -DXVSDK_INCLUDE_DIRS="/path/to/xvsdk/include" -DXVSDK_LIBRARIES="/path/to/libxv-sdk.so"
	source devel/setup.bash
	roslaunch xv_sdk xv_sdk.launch

### Unit Tests

~~Run the unit tests with~~

	catkin_make run_tests_xv_sdk

### Static code analysis

~~Run the static code analysis with~~

	catkin_make roslint_xv_sdk

## Usage

Run (only once at a time!) the xv_sdk node with:

    cd path/to/catkin_workspace
    source devel/setup.bash
    roslaunch xv_sdk xv_sdk.launch
  
To display the color camera, run in an other terminal:

    rosrun image_view image_view image:=/xv_sdk/xv_dev/color_camera/image_color

A demo can be run in an other terminal with:

    rosrun rviz rviz -d `rospack find xv_sdk`/rviz/demo.rviz
    
To display more features, click "Add" at the bottom of the "Displays" panel.

To display the tree of coordinate frames, run in an other terminal:

    rosrun rqt_tf_tree rqt_tf_tree


## Config files

* **default.yaml** default config file


## Launch files

* **xv_sdk.launch:** launches the XVisio SDK

     Arguments:

     - **`param_file`** (optional) custom parameter `.yaml` file

## Nodes

### xv_sdk

Runs the XV-SDK C++ API in the background and expose it through ROS services and topics.

#### Subscribed Topics

none

#### Published Topics

Additional topics are published by some ROS packages like `camera_info_manager`, `image_transport` or
`ddynamic_reconfigure`. To see the complete list, run `rostopic list` while running `xv_sdk` and having
a compatible device connected.

* **`/xv_sdk/new_device`** ([std_msg/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html))

  Informs about new devices being plugged. By default, the parameter `uuid_length` is set to zero,
  which does not allows multiple devices to be handled at the same time. In that case, this message
  will just contain the string `xv_dev`. But, if `uuid_length` is set 1 or more, the message will
  contains a string formated as follow: `"uuid_#"`,	where `#` is a sub-part of size `uuid_length` of
  the alpha-numeric serial number of the device where characters that are not supported inside ROS
  namespaces have been replaced by an underscore '_'. For example, you can monitor new devices in
  the console with:

		rostopic echo /xv_sdk/new_device
	
* **`/xv_sdk/xv_dev/slam/pose`** ([geometry_msgs/PoseStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html))

  Informs about the current 6DOF pose of the device's IMU, at IMU rate, inside the mixed-mode slam map.

* **`/xv_sdk/xv_dev/slam/stereo_planes`** ([msgs/Planes](.msg/Planes.msg))

  Informs about 3D planes detected in the sparse 3D point cloud of the mixed-mode slam.
  
* **`/xv_sdk/xv_dev/slam/tof_planes`** ([msgs/Planes](.msg/Planes.msg))

  Informs about 3D planes detected in the dense 3D point cloud of the ToF camera in the mixed-mode slam map frame.

* **`/xv_sdk/xv_dev/imu_sensor/data_raw`** ([sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html))

  Informs about the last IMU measurement at IMU rate.
  
* **`/xv_sdk/xv_dev/imu_sensor/orientation`** ([msgs/Orientation](.msg/Orientation.msg))

  Informs about the current 3DOF rotation of the device's IMU, at IMU rate, using IMU-only filtering.

* ~~**`/xv_sdk/xv_dev/fisheye_cameras/images`** ([msg/FisheyeImages](./msg/FisheyeImages.msg))~~

  Informs about wide-angle/grayscaled/time-synchronized images captured by the device at camera rate.

* **`/xv_sdk/xv_dev/fisheye_cameras/left/image`** ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))
* **`/xv_sdk/xv_dev/fisheye_cameras/right/image`** ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

	Informs about the left (respectively right) wide-angle/grayscaled image captured by the
  device at camera rate. If the device has more than 2 fisheye cameras, those topics only
  publish the images of the "main" stereo pair.
  
* **`/xv_sdk/xv_dev/color_camera/image_color`** ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

  Informs about RGB images captured by the device at camera rate.
  
* **`/xv_sdk/xv_dev/tof_camera/image`** ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

  Informs about ToF images captured by the device at camera rate. Type is 32bits float in meters.
  
* **`/xv_sdk/xv_dev/tof_camera/image_color`** ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

  Estimated color associated with depth measurements taken from the previous color image of the color camera.
  Better result if the mixed-mode visual slam is running (ToF and color camera are not time-synchronized,
  so the device motion needs to be corrected).
  
* **`/xv_sdk/xv_dev/tof_camera/point_cloud`** ([sensor_msgs/Pointcloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/Pointcloud2.html))

  Colored 3D point cloud raycasted in the mixed-mode slam frame (better color and spacial positionning if slam running).
	
#### Services

Additional services are made available by some ROS packages like `camera_info_manager`, `image_transport` or
`ddynamic_reconfigure`. To see the complete list, run `rosservice list` while running `xv_sdk` and having
a compatible device connected.

* **`/xv_sdk/get_devices`** ([srv/GetDevices.srv](./srv/GetDevices.srv))

	Returns the list of devices as a list of strings containing the namespaces associated to plugged
	devices (see `/xv_sdk/new_device` naming convention). For example, you can call this service from the
	console with

		rosservice call /xv_sdk/get_devices "{}"

* **`/xv_sdk/xv_dev/slam/start`** ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))
* **`/xv_sdk/xv_dev/slam/stop`** ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

  Starts/stops the XVisio mixed-mode visual slam. Stopped by default.

* **`/xv_sdk/xv_dev/slam/get_pose`** ([srv/GetPose.srv](./srv/GetPose.srv))

  Returns the predicted pose of the device' IMU using the given prediction duration.

* **`/xv_sdk/xv_dev/slam/get_pose_at`** ([srv/GetPoseAt.srv](./srv/GetPoseAt.srv))

  Returns the estimated pose of the device's IMU at the given time.
  
* **`/xv_sdk/xv_dev/imu_sensor/get_orientation`** ([srv/GetOrientation.srv](./srv/GetOrientation.srv))

  Returns the predicted 3Dof rotation of the device's IMU using the given prediction duration (IMU-only filtering).

* **`/xv_sdk/xv_dev/imu_sensor/get_orientation_at`** ([srv/GetOrientationAt.srv](./srv/GetOrientationAt.srv))

  Returns the estimated 3Dof rotation of the device's IMU at the given time (IMU-only filtering).
  
* **`/xv_sdk/xv_dev/imu_sensor/start_orientation`** ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))
* **`/xv_sdk/xv_dev/imu_sensor/stop_orientation`** ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

  Starts/stops the embedded computation of the 3Dof rotation (IMU-only filtering). Started by default.
  
* **`/xv_sdk/xv_dev/color_camera/start`** ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))
* **`/xv_sdk/xv_dev/color_camera/stop`** ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

  Starts/stops the color camera of the device. Started by default.
  
* **`/xv_sdk/xv_dev/tof_camera/start`** ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))
* **`/xv_sdk/xv_dev/tof_camera/stop`** ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

  Starts/stops the ToF camera of the device. Started by default.

#### Parameters

* **`uuid_length`**

  Defines the number of chars put in place of the `#` in the device-specific ROS namespace `uuid_#`.
  By default, it is set to zero, which does not allows multi-device handling. In that case, `uuid_#`
  is just replaced by `xv_dev` like in all examples provided in this README.

Some other parameters can be changed on the fly using the `dynamic_reconfigure` package. A graphic interface
is available by running `rosrun rqt_reconfigure rqt_reconfigure`.

In the console, you can get the list of configurable nodes with `rosrun dynamic_reconfigure dynparam list`.

For example, to get the available parameters of a node, run:

    rosrun dynamic_reconfigure dynparam get /xv_sdk/xv_dev/fisheye_cameras

For example, to force the `fisheye_cameras` to have a constant exposure time of 2560µs, run:

    rosrun dynamic_reconfigure dynparam set /xv_sdk/xv_dev/fisheye_cameras exposure 2560

Another example, to get the ToF point cloud expressed in the local ToF frame (and not the world frame), run:

    rosrun dynamic_reconfigure dynparam set /xv_sdk/xv_dev/tof_camera use_map_frame false
    
For more info, see [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure).

#### Frames and tf transforms

By default, all provided frames have a `tf_prefix` set to `xv_dev` (single device case) or of the
form `uuid_#/` (multi-device case). They can be customized using the dynamic configuration. Default
names are as follows:

* **`map_optical_frame`** the standard ROS "map" frame but with optical convention axes
* **`imu_optical_frame`** the standard ROS "imu_link" frame but with optical convention axes
* **`imu_optical_frame_unfiltered`** same as imu_optical_frame but directly provided by the raw visual slam
* **`fisheye_left_optical_frame`** optical frame of the left fisheye
* **`fisheye_right_optical_frame`** optical frame of the right fisheye
* **`color_optical_frame`** optical frame of the color camera
* **`tof_optical_frame`** optical frame of the ToF camera

Those static transforms are defined to ease integration with other ROS packages:

* **`(without tf_prefix/)map` -> `map`** this allows each device to have its own `.../map` positionned in the standard ROS `map` frame
* **`map` -> `map_optical_frame`** same origin, different axes convention
* **`imu_optical_frame` -> `imu_link`** same origin, different axes convention
* **`imu_optical_frame` -> `base_link`** same origin, different axes convention (`imu_link` == `base_link`)

Those static transforms are taken from the internal extrinsic calibration of the device:

* **`imu_optical_frame` -> `fisheye_left_optical_frame`** left fisheye extrinsic
* **`imu_optical_frame` -> `fisheye_right_optical_frame`** right fisheye extrinsic
* **`imu_optical_frame` -> `color_optical_frame`** color camera extrinsic
* **`imu_optical_frame` -> `tof_optical_frame`** ToF camera extrinsic

Those dynamic transforms are provided by the XV-SDK visual-slam algorithm:

* **`map_optical_frame` -> `imu_optical_frame`**

  This localization is provided by the slam algorithm, filtered with imu measurements.

* **`map_optical_frame` -> `imu_optical_frame_unfiltered`**

  This localization is directly provided by the visual slam algorithm. It has lower frequency,
  higher jitter and higher latency than the filtered localization.
  
#### Multi-device support

To be able to connect multiple XV-SDK compatible devices at the same time, the parameter `uuid_length`
of the node `xv_sdk` has to be set to 1 or more (4 should be enought to avoid any uuid collision).
By default, `uuid_length` is set to zero, which, for example, make the `xv_sdk` node publish:

* **topics** like: `/xv_sdk/xv_dev/color_camera/image_color`
* **services** like: `/xv_sdk/xv_dev/color_camera/start`
* **transforms** like: `xv_dev/color_optical_frame`

If `uuid_length` is set to 4, then we will get something like:

* **topics** like: `/xv_sdk/uuid_1a1a/color_camera/image_color`
* **services** like: `/xv_sdk/uuid_1a1a/color_camera/start`
* **transforms** like: `uuid_1a1a/color_optical_frame`

The `uuid_length` parameter is not dynamic. It is loaded at the `xv_sdk` node creation. One way of
overriding the default value (zero) is to provide a `.yaml` configuration file. A one-liner could be:

    echo -e "uuid_length: 4" > /tmp/config.yaml && roslaunch xv_sdk xv_sdk.launch param_file:=/tmp/config.yaml

## Bugs & Feature Requests

Please report bugs and request features using the ~~[Issue Tracker]()~~.

## Todo list

* Fix workaround for inverted RGB colors
* Fix crash bad fisheye timestamp
* convert XVSDK timestamps to ROS time
* CSLAM api (http://wordaligned.org/articles/cpp-streambufs)
* objects detection (https://github.com/stereolabs/zed-ros-wrapper/blob/master/zed_interfaces/msg/ObjectStamped.msg)
* use topic imu_sensor/data with orientation instead of topic imu_sensor/orientation ?
* threads for each heavy computation instead of doing it inside callbacks
* publish synced fisheyes ?
* diagnostic_updater ?
* nodelet ?
* URDF ?
* 2d occupancy grid demo ?
* stereo_msgs/DisparityImage for sgbm ?
* index this ROS package ? (http://wiki.ros.org/rosdistro/Tutorials/Indexing%20Your%20ROS%20Repository%20for%20Documentation%20Generation)
* ~~rename node xvsdk_ros ?~~ naming convention is not so clear


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
