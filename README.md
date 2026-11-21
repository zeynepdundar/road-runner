# RoadRunner

Vision-based autonomous lane following for a TurtleBot (Kobuki base) in a custom Gazebo track, built with ROS Kinetic, C++ and OpenCV.

The robot reads frames from its Kinect RGB camera, finds the two lane markings, fits a curve to each, and steers with a PID controller to stay in the middle of the lane.

Developed as the term project for **BLG436E – Robotics** at Istanbul Technical University (Fall 2020).

## How it works

```
camera/rgb/image_raw ──► image_processor_node ──► /cmd_vel_mux/input/navi
                          │
                          ├─ 1. Grayscale + Gaussian blur
                          ├─ 2. Canny edge detection
                          ├─ 3. Perspective warp to a bird's-eye view
                          ├─ 4. Histogram of the bottom strip → two lane start windows
                          ├─ 5. Sliding-window search up the image for each lane
                          ├─ 6. Least-squares quadratic fit per lane (x = ay² + by + c)
                          ├─ 7. Error = image center − midpoint of the two lanes at the bottom row
                          └─ 8. PID on the error → angular velocity
```

- **Lane detection** – `calculateDistanceToLaneCenter()` in `image_processor.cpp`. The fitted curves and search windows are drawn in a live `projection` OpenCV window for debugging.
- **Steering** – `pid_controller.h`, a PID controller with integrator clamping (Kp = 0.009, Kd = 0.01, Ki = 0 by default).
- **Speed states** – the node has three drive states (`0` halt, `1` 50% speed, `2` full speed; base linear speed 0.5 m/s). The track has matching *stop*, *slow* and *forward* sign models, meant to switch between these states.

## Repository layout

```
RoadRunner/
├── image_processor/          # ROS package: lane detection + control node
│   └── src/
│       ├── image_processor.cpp   # main node (vision pipeline + PID control)
│       ├── pid_controller.h      # PID controller
│       ├── polynomial_fit.h      # least-squares polynomial regression (MIT, Chris Engelsma)
│       └── state_publisher.py    # stub node for publishing the drive state
└── mycustomworld/            # ROS package: Gazebo track and launch file
    ├── course.launch         # starts Gazebo with the track + TurtleBot (Kobuki, Kinect)
    ├── course.world          # the lane track world
    ├── road.material, *.png  # road texture
    └── models/               # stop / slow / forward traffic sign models
```

## Requirements

- Ubuntu 16.04
- ROS Kinetic (desktop-full, includes Gazebo 7 and OpenCV)
- TurtleBot simulation packages: `turtlebot_gazebo`
- `cv_bridge`, `image_transport`

```bash
sudo apt install ros-kinetic-turtlebot-gazebo ros-kinetic-cv-bridge ros-kinetic-image-transport
```

## Build

Clone the repository into the `src` folder of a catkin workspace:

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/zeynepdundar/road-runner.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run

Terminal 1: start the simulation (track + robot):

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find mycustomworld)/models
roslaunch mycustomworld course.launch
```

Terminal 2: start lane following:

```bash
rosrun image_processor image_processor_node
```

The robot starts driving at full speed. The bird's-eye view with the fitted lanes appears in the `projection` window, and the steering error is printed to the terminal.

## Team

- Seymur Kafkas – [@seymurkafkas](https://github.com/seymurkafkas)
- Zeynep Dündar – [@zeynepdundar](https://github.com/zeynepdundar)
- _<other team members>_

Originally developed in [seymurkafkas/RoadRunner](https://github.com/seymurkafkas/RoadRunner).

## Credits

- `polynomial_fit.h` – polynomial regression by Chris Engelsma, MIT License.
- Traffic sign models – Mariyam Fedoseeva (TUM).
