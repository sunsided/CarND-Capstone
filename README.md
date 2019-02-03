# CarND Capstone

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: 
Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

![](imgs/final-project-ros-graph-v2.png)

## Installation

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) if you have Ubuntu 18.04.
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/sunsided/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing

1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) (see [Bags](http://wiki.ros.org/Bags)) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

## Project layout

The main project code can be found in `./ros/src`. This directory contains the following
ROS packages:

### `./ros/src/tl_detector/`

This package contains the traffic light detection node: `tl_detector.py`. 
This node takes in data from the `/image_color`, `/current_pose`, and `/base_waypoints` 
topics and publishes the locations to stop for red traffic lights to the 
`/traffic_waypoint` topic.

![](imgs/tl-detector-ros-graph.png)

The `/current_pose` topic provides the vehicle's current position, and 
`/base_waypoints` provides a complete list of waypoints the car will be following.

Traffic light detection should take place within `tl_detector.py`, whereas traffic light classification should take place within `../tl_detector/light_classification_model/tl_classfier.py`.

### `./ros/src/waypoint_updater/`

This package contains the waypoint updater node: `waypoint_updater.py`. 
The purpose of this node is to update the target velocity property of each waypoint 
based on traffic light and obstacle detection data. This node will subscribe to the 
`/base_waypoints`, `/current_pose`, `/obstacle_waypoint`, and `/traffic_waypoint` topics, 
and publish a list of waypoints ahead of the car with target velocities 
to the `/final_waypoints` topic.

![](imgs/waypoint-updater-ros-graph.png)

### `./ros/src/twist_controller/`

Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake,
and steering have electronic control. This package contains the files that are
responsible for control of the vehicle: the node `dbw_node.py` and the file
`twist_controller.py`, along with a pid and lowpass filter that you can use in your 
implementation. The `dbw_node` subscribes to the `/current_velocity` topic along with the 
`/twist_cmd` topic to receive target linear and angular velocities.
Additionally, this node will subscribe to `/vehicle/dbw_enabled`, which indicates if
the car is under dbw or driver control. This node will publish throttle, brake, and
steering commands to the `/vehicle/throttle_cmd`, `/vehicle/brake_cmd`,
and `/vehicle/steering_cmd` topics.

![](imgs/dbw-node-ros-graph.png)

### Additional packages

In addition to the above packages you will find the following, which are not necessary
to change for the project. The `styx` and `styx_msgs` packages are used to provide a
link between the simulator and ROS, and to provide custom ROS message types:

#### `./ros/src/styx/`

A package that contains a server for communicating with the simulator,
and a bridge to translate and publish simulator messages to ROS topics.

#### `./ros/src/styx_msgs/`

A package which includes definitions of the custom ROS message types used in the project.

#### `./ros/src/waypoint_loader/`

A package which loads the static waypoint data and publishes to `/base_waypoints`.

#### `./ros/src/waypoint_follower/`

A package containing code from [Autoware](https://github.com/CPFL/Autoware)
which subscribes to `/final_waypoints` and publishes target vehicle linear
and angular velocities in the form of twist commands to the `/twist_cmd` topic.

## Suggested Order of Project Development

Because development spans several packages with some nodes depending on messages
published by other nodes, it is suggested to complete the project in the following order:

1. **Waypoint Updater Node (Partial):** Complete a partial waypoint updater which 
   subscribes to `/base_waypoints` and `/current_pose` and publishes to `/final_waypoints`.
2. **DBW Node:** Once the waypoint updater is publishing `/final_waypoints`, 
   the waypoint_follower node will start publishing messages to the `/twist_cmd` topic.
   At this point, we have everything needed to build the `dbw_node`.
   After completing this step, the car should drive in the simulator, 
   ignoring the traffic lights.
3. **Traffic Light Detection:** This can be split into 2 parts:
    * **Detection:** Detect the traffic light and its color from the `/image_color`.
      The topic `/vehicle/traffic_lights` contains the exact location and status of all
      traffic lights in simulator, so you can test your output.
    * **Waypoint publishing:** Once traffic lights are correctly identified the their position
      is determined, we can convert it to a waypoint index and publish it.
4. **Waypoint Updater (Full):** Use `/traffic_waypoint` to change the waypoint target
   velocities before publishing to `/final_waypoints`. The car should now stop at red 
   traffic lights and move when they are green.


## Topics and message types

| Topic               | Message Type                | Notes                                                                                                                  |
|---------------------|-----------------------------|------------------------------------------------------------------------------------------------------------------------|
| `/base_waypoints`   | `styx_msgs/Lane`            | Waypoints as provided by a static `.csv` file.                                                                         |
| `/current_pose`     | `geometry_msgs/PoseStamped` |  Current position of the vehicle, provided by the simulator or localization.                                           |
| `/final_waypoints`  | `styx_msgs/Lane`            | This is a subset of `/base_waypoints`. The first waypoint is the one in `/base_waypoints` which is closest to the car. |
| `/closest_waypoint` | `waypoint_updater/WaypointLocation`            | Provides the world coordinates of the closest waypoint and the index into the list published on `/base_waypoints`. |

### Closest waypoint

The `/closest_waypoint` topic publishes the closest waypoint in world coordinates,
as well as its index into the list published on the `/base_waypoints` topic.

```yaml
header: 
  seq: 1679
  stamp: 
    secs: 1549213758
    nsecs: 328206062
  frame_id: "/world"
index: 347
pose: 
  position: 
    x: 1202.23
    y: 1187.59
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.023799069622
    w: 0.999716762031
```

To see it in action, run

```sh
rostopic echo /closest_waypoint
```

## Handling ROS

If [direnv](https://direnv.net/) is installed and set up, the calls to
`source devel/setup.bash` can be skipped in the described commands.

From the `./ros` directory, run the following two commands to build the project
and source the environment:

```sh
catkin_make
```

Start the ROS master in a separate terminal:

```sh
source devel/setup.bash
roscore
```

Launch the system using `./run` (in the `./ros` subdirectory), or execute:

```sh
source devel/setup.bash
roslaunch launch/styx.launch
```

In yet another terminal, source the environment again and obtain a topic list
using `rostopic`:

```sh
source devel/setup.bash
rostopic list
```

You can now inspect individual topics, e.g. `/final_waypoints`

```sh
rostopic info /final_waypoints
```

This should give us a description indicating `Type: styx_msgs/Lane`. Let's inspect the
message `styx_msgs/Lane` using `rosmsg`:

```sh
rosmsg info styx_msgs/Lane
```

This should give us the following output:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
styx_msgs/Waypoint[] waypoints
  geometry_msgs/PoseStamped pose
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
  geometry_msgs/TwistStamped twist
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Twist twist
      geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z
```

Camera images are published on the `/image_color` topic
and can be observed using e.g.

```sh
rqt_image_view /image_color
```

To see the output of the traffic light detection module, run

```sh
rostopic echo /traffic_waypoint
```

## Capturing camera images

Camera images are published on the `/image_color` topic. The `tl_recorder` script
in the `tl_detector` package captures these images and stores them to disk.

To run the script, issue

```sh
rosrun tl_detector tl_recorder
```

on a shell while both simulator and system are running and camera input is enabled in the simulator.
