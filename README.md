# ECE 148 Spring 2025 Team 11 Final Project
## "Balloon Animal Detection"

<img src="assets/jsoe.png" alt="jacobs_school_of_engineering" style="width:50vw;">
<img src="assets/carwithballoons.jpg" alt="car with balloon animals" style="width: 50vw;">

---

[Final Presentation](https://docs.google.com/presentation/d/1dCoKgqVfZQMhyggTEogI8tUQDtrG1OyyUPlr59H-XTo/edit?usp=sharing)

## Project Description
We have built a car that autonomously recognizes randomly placed target balloon animals. It responds differently depending on the identified animal, all while following a designated path in a ***controlled and optimal manner***.

### Main Objectives
- Detection of various balloon animals via Roboflow object detection model
- Different actions based on which balloon animal is detected
- Speaker plays sounds corresponding to each balloon animal
- Above nodes running in parallel with OpenCV lane following

### Nice to have
- LiDAR integration for more refined movement

## Members
| Name | Major | Year |
|------|-------|------|
|Jose Arreguin|Mathematics:Computer Science|SP25|
|Kenny Huh|Mechanical Engineering|SP25|
|Leonardo Hartl|Mechanical Engineering|SP25|
|Nicolas Ventos|Aerospace Engineering|SP25|

# Demo Video

[![Watch the video](https://img.youtube.com/vi/k4bPpsPWAw4/hqdefault.jpg)](https://www.youtube.com/watch?v=k4bPpsPWAw4)

# Design Components

| Component | Designer |
|-----------|----------|
|Speaker Mount|Nicolas|
|Dual Camera Mount|Kenny|
|Camera Extension|Kenny|

## Component Images
**Speaker Mount Design**

<img src="assets/speakermount.png" alt="speaker_mount_design" style="width: 35vw;">

**Dual Camera Mount Design**

<img src="assets/dualcamera.png" alt="dual_camera_mount_design" style="width: 35vw;">

**Camera Extension Design**

<img src="assets/camerabase.png" alt="camera_extension_design" style="width: 35vw;">

## Wiring

<img src="assets/wiring.png" alt="wiring_diagram_for_car" style="width: 50vw;">

# Software

## System Architecture Overview

<img src="assets/ros2.png" alt="ros2_archtecture" style="width: 65vw;">

### Implementation Details
We created our own package named ucsd_robocar_balloon2_pkg that contains our main nodes (balloon, speaker and vesc)

#### `balloon_node.py`
- handles the balloon detection with the OAKD camera specified by its MXID (bottom camera).
- MXID passed through the launch file parameters
- *publishes* to the *balloon_detections* topic, which baloon animal it detected or *none* if it has detected nothing

#### `speaker_node.py`
- *subscribes* to the *balloon_detections* topic, which using the recieved detected animal, plays the corresponding audio via the aplay command:
```
try: 
  subprocess.run(
    ['aplay', '-D', 'plughw:2,0', sound_path],
    check=True
  )
except subprocess.CalledProcessError as e:
  self.get_logger().error(f"Failed to play audio: {e}")
```

#### `vesc_node.py`
- *subscribes* to the *balloon_detections* topic and recieves the detected animal.
- It then *publishes* **pause** to the *lane_guidance_control* which switches the state of the lane guidance node to halt sending values to the VESC.
- Then it controls the car in a specified manner:

<img src="assets/paths.png" alt="paths_for_each_animal" style="width: 50vw;">

- Finally it *publishes* **resume** to the *lane_guidance_control* topic which reverts the lane guidance node to a normal state.

#### `multi_cam_node.py`
- configured to only use one camera specified by its MXID (top camera).
- We had to run this every time before launch:
```
cd /home/projects/sensor2_ws
colcon build --packages-select multi_cam
source install/setup.bash
build_ros2
```

#### `lane_guidance_node.py`
- *subscribes* to the *lane_guidance_control* and switches its state depending on whether it recieves a **pause** or a **resume**. 
- In the pause state it does not send commands to the vesc. 
- In resume state it works as normal.

## Course Deliverables

**[Outside Lane Following](https://youtu.be/QxAHrUlSgUk?si=JCECQAlf1tRa1Ojq)**

**[Inside Lane Following](https://youtu.be/GkR7rY1FQCk?si=l_rVWvA1lEhHj7bw)**

**[GPS Navigation](https://youtu.be/qnhPcpjn2ac)**

**[DonkeyCar 3 Laps EBUII](https://youtube.com/shorts/WQX3FQiDqeY)**

**[Model with Roboflow](https://youtube.com/shorts/SliKKk7VkHU?si=tkCcVwuUGS_EaLJ8)**

**[Model with Luxonis SDK](https://youtu.be/QeoocV0fp0k?si=ojEsEoA7VQYCrGZg)**

# Gantt Chart

<img src="assets/gantt.jpg" alt="gantt_chart_final" style="width: 85vw;">

# Gallery

<img src="assets/closeup.jpg" alt="close_up_of_car" style="width: 50vw;">

<img src="assets/leftside.jpg" alt="left_side_of_car" style="width: 50vw;">

<img src="assets/leo.jpg" alt="lab_picture" style="width: 50vw;">

