This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Cargo team and team members
* Team name
#### Cargo
*Cargo* means car drives autonomously.
* Team members

|                  | Name            |  Email                  |
| --------         | -----           |  ----                   |
| Team Lead        | Fuqiang Xu      |  qiyuwang@163.com       |
| Team Member 1    | Sridhar Sampath |  sridhar912@gmail.com   |
| Team Member 2    | Takashi Ikegami |  tks.ikegami@gmail.com  |
| Team Member 3    | Andras Hejj     |  andreas.hejj@gmail.com |
| Team Member 4    | Qitong Hu       |  huqitong@aiztone.com   |

### Introduction

#### For Carla test

In slack channel, a lot of classmates discussed about the difference between simulator and Carla.

By the way, it is hard to tune PID parameters to get linear distribution in small range from `-max_braking_percentage` to `max_throttle_percentate`.

According to these information, we changed control from PID to linear control and set different throttle limit and brake limit for Simulator and Carla in launch files.

    if proposed_linear_velocity > current_linear_velocity:
        if delta_linear_velocity / proposed_linear_velocity > 0.3:
            throttle = self.max_throttle_percentage
        else:
            throttle = max((delta_linear_velocity / proposed_linear_velocity)/0.3*self.max_throttle_percentage, self.max_throttle_percentage)      
    elif current_linear_velocity > 1:
        brake = 3250*self.max_braking_percentage*-1
    else:
        brake = 3250*0.01

#### Brake torque calculation

It is necessary to calculate brake torque with the following since Brake CMD type is `TORQUE`.

`Brake torque = (vehicle_mass + fuel_capacity * GAS_DENSITY) * deceleration *  wheel_radius`

The above simple equation is based on assumption that the ground can absolutely provide enough friction force for tires. Anyway this is enough for our case here. 

According to `<param name="decel_limit" value="-1." />` in `dbw.launch`, the estimated maximum brake torque for Carla is about 428Nm, only 0.13 times of Carla maximum working brake torque 3250Nm.

In this case, `max_throttle_percentate` and `max_braking_percentage` for Carla are assigned 0.1 in dbw.launch.

    <param name="max_throttle_percentage" value="0.1" />
    <param name="max_braking_percentage" value="-0.1" />

 `max_throttle_percentate` and `max_braking_percentage` for Simulator are assigned 0.8 in dbw_sim.launch.

    <param name="max_throttle_percentage" value="0.8" />
    <param name="max_braking_percentage" value="-0.8" />

#### Project workflow

  * Waypoint Updater Node (Partial)
  * DBW Node
  * Traffic light classifier model choose and training
  * Detection
  * Traffic Waypoint publishing
  * Waypoint Updater (Full)
  * Intergration and bug fix

### Summary

*This is a exciting and not easy project.*

#### Inception model

The model we use to traffic light detection is transferred from Tensorflow objecct detection API ssd-inception model.

There are two separately classifier model, one for simulator and one for Carla. All these parameters are integrated in tl_detector.launch and tl_detector_site.launch.

The original pre-trained model is downloaded from tensorflow/models. One data are from Bosch and the other from our classmate Shyam Jagannathan shared in slack, which is taken from simulator and rosbag.

We use the Tensorflow Object-Detection API to train the model with some hyperparameters tuned in the config file and a project-specific label-map file (number of classes to 4, proposal region to 10 and second stage batch size to 8, max detection to 4 and max per class to 4). we get the models after training around 20K step with final loss under 0.5.

At first, we train the model with Bosch data (rgb version and additional set) but traffic light detection perform poor and takes a lot of time. Then we train with task-specific data only.

We tried with ssd-mobile, ssd-inception, faster-rcnn and rcfn as pre-trained model separately. The later-two are good at accuracy but the model size is over 100M and need additional 0.03s (about 60% more) time to detect. We compare these four models from the processing time, accuracy and model size, finally we decide to use ssd-Inception model.

The traffic light classifier loads the model during initiating, then processes the image and outputs the classification and probability of detected box. We set 0.5 as the accepting threshold, and all the accepted ones will have a majority vote, the vote result will be our final judgment. The whole process takes around 0.05s for ssd-Inception and 0.08s for Faster-RCNN model (both testing on Qitong Ubuntu16.04 system with GTX1080Ti), both satisfactory with designed requirement time to response -- 0.1s (10Hz).

There are some important tips shown blow.
1. Speed Limit
  * 40 km/h for simulator
  * 10 km/h for Carla

These above information can be gotten from `ros/src/waypoint_loader/launch`.
2. Brake CMD Type

A lot of people including me feel hard to make vehicle stop in simulator. The reason is that we always ignoring Brake CMD Type setting.

The default `throttle cmd`  type is `ThrottleCmd.CMD_PERCENT`.

The default `brake cmd` type is `BrakeCmd.CMD_TORQUE`.

All these can be found in `/ros/src/twist_controller/dbw_node.py`.

Some useful information can be found in '/opt/ros/kinetic/share/dbw_mkz_msgs/msg/BrakeCmd.msg'

    # Brake pedal
    # Opt ions defined below
    float32 pedal_cmd
    uint8 pedal_cmd_type

    #  Brake On Off (BOO), brake lights
    bool boo_cmd

    # Enable
    bool enable

    # Clear driver overrides
    bool clear

    # Ignore driver overrides
    bool ignore

    # Watchdog counter (optional)
    uint8 count

    uint8 CMD_NONE=0
    uint8 CMD_PEDAL=1   # Unitless, range 0.15 to 0.50
    uint8 CMD_PERCENT=2 # Percent of maximum torque, range 0 to 1
    uint8 CMD_TORQUE=3  # Nm, range 0 to 3250

    float32 TORQUE_BOO=520  # Nm, brake lights threshold
    float32 TORQUE_MAX=3412 # Nm, maximum torque

### Acknowledgement
Thanks a lot for each team members' hard work.

*Especially thanks Qitong Hu's excellent work on Tensorflow object detection API model training for traffic light detection.*

Thanks a lot for support from *John Chen*、 *Anthony Sarkis* and others classmates from slack.

### Some Failure Experiences

1. Blas SGEMM and cuDNN Error

Sometimes there are Blas SGEMM Internal Error or cuDNN can not launch on my xps 9560 Ubuntu 16.04 system, but most of time there are no problem. These problems also disappear with repeating program again.  

Qitong confirmed that there are no problem on his PC Ubuntu system.

Our conclusion is that these 2 problems may be caused by xps 9560 CUDA and cuDNN library installation.

2. Brake CMD Type Percent

An interesting issue is that the car in simulator can not stop before traffic light when switches to Brake CMD Type from `TORQUE` to `PERCENT`, through John Chen team Vulture seems no problem.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 127.0.0.1:4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

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
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
