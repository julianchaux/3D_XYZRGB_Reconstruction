# 3D XYZRGB Reconstruction

This repository contains the description of a system that performs the extraction of geometric and color characteristics of a navigation environment, by using a reconstruction of a map with six dimensions of information: X, Y, Z, R, G and B.

![Image1](https://github.com/julianchaux/3D_XYZRGB_Reconstruction/blob/master/data/3D%20colorless%20reconstruction.png)

## Folders and files

Repository folders and files:

```
/yourRoot    -path
  |--  Readme.md
  |-- src # source codes
	|-- Python source codes     # main source codes
		|-- fixed_no_color.py # Point cloud generation: fixed without color
		|-- scan_and_go_no_color.py # Dynamic reconstruction without color for the scanning mode: "scan and go"
		|-- stop_and_scan_no_color.py # Dynamic reconstruction without color for the scanning mode: "stop and scan"
		|-- scan_and_go_color.py # Dynamic color reconstruction for the scanning mode: "scan and go"
		|-- stop_and_scan_color.py # Dynamic color reconstruction for the scanning mode: "stop and scan"
		|-- send_command.py # Manual sending of Command
		|-- mover_ax12.py # Manual movement of the servomotor
  |-- data
    |-- pointcloud_1.txt # Database file for the "scan and go" point cloud without color
    |-- pointcloud_2.txt # Database file for the "stop and scan" point cloud without color
    |-- pointcloud_3.txt # Database file for color point cloud "scan and go"
    |-- pointcloud_4.txt # Database file for color point cloud "stop and scan"
```

## Hardware requirements
| Component | Image | Reference |
| -------- | ------- | --------- |
| **Turtlebot 2** | ![Turtlebot image](https://www.turtlebot.com/assets/images/turtlebot_2_lg.png) | [https://www.turtlebot.com/turtlebot2/](https://www.turtlebot.com/turtlebot2/) |
| **Dynamixel AX12A servomotor** | ![Dynamixel AX12](https://emanual.robotis.com/assets/images/dxl/ax/ax-12+_product.png) | [https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/) |
| **LOGITECH C920S HD PRO RGB Camera** | ![LOGITECH C920S HD PRO RGB Camera](https://assets.logitech.com/assets/54515/hd-webcam-pro-c920-gallery.png) | [https://www.logitech.com/en-us/product/hd-pro-webcam-c920](https://www.logitech.com/en-us/product/hd-pro-webcam-c920) |
| **HOKUYO RangeFinder URG-04LX-UG01** | ![HOKUYO URG-04LX-UG01](https://www.hokuyo-aut.jp/p/product/20160223110957_85.jpg) | [https://www.hokuyo-aut.jp/search/single.php?serial=166](https://www.hokuyo-aut.jp/search/single.php?serial=166) |

## Software requirements

- Ubuntu 16.04.6 LTS (Xenial Xerus)
- ROS distribution: ROS Kinetic Kame [http://wiki.ros.org/kinetic](http://wiki.ros.org/kinetic)
- ROS nodes/drivers libraries:
	-   [Turtlebot](http://wiki.ros.org/turtlebot_bringup?distro=kinetic)
	-  [Dynamixel servomotor](http://wiki.ros.org/dynamixel)
	-  [Hokuyo](http://wiki.ros.org/urg_node)
	-  [Camera](http://wiki.ros.org/usb_cam)
- Python 3.6 in this case I used [Visual Studio Code](https://code.visualstudio.com/)
-   OpenCV 4

## ROS Installing.

For ROS install you must follow the next steps:

-   [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)

or follow the video:

-   [https://www.youtube.com/watch?v=36O6OGOJG1E](https://www.youtube.com/watch?v=36O6OGOJG1E)

## Use ROS

In a terminal you should run the ROS kernel with

`roscore`

in other tabs the following commands are executed, each in a separate terminal:

- `roslaunch turtlebot_bringup minimal.launch`
- `roslaunch control_dynamixel_ax12 controller_manager.launch`
- `roslaunch control_dynamixel_ax12 start_motor_controller.launch`
- `roslaunch usb_cam usb_cam test.launch`
- `rosrun urg_node urg_node`
- `python send_command.py`

The proposed application contains two (2) modes of reconstruction.  The first one is called "`Scan and go`", is a continuous mode where the  robot  doesn’t  need  to  slow  down  during  its  trajectory. The  servomotor  on  which  the  LiDAR  sensor  is  supported operates  with  a  fixed  tilt  angle,  e.g.  with  a  45  degrees  of incidence in the central beam with respect to the floor.  Therefore the command is executed to position the servomotor at the required angle using the following python script:

`python mover_ax12.py`

Then the next python script is executed to start the reconstruction in "scan and go" mode.

`python scan_and_go_color.py` or `python scan_and_go_no_color.py`

This scan mode requires the pilot to move the robot manually in the directions that require rebuilding, therefore, the following command must be executed to tele-operate the robot.  

`roslaunch turtlebot_teleop keyboard_teleop.launch` or `roslaunch turtlebot_teleop ps3_teleop.launch`

When the path is finished, the environment map has been reconstructed, the text files are generated that save the X, Y, Z, R, G, B information of the environment map.  An example of the generated files is in the `data` folder. A 3D environment generated in this reconstruction mode is shown below.

![Image2](https://github.com/julianchaux/3D_XYZRGB_Reconstruction/blob/master/data/ColorlessPointCloud1.jpg)

The second mode of reconstruction is called "`Stop and scan`", this mode stops  the  robot  from  moving  at  some  points  wherethe pilot considers it, to perform scans with the servo motorin a range of inclinations from +45 degrees to -45 degrees, a common practice in fixed scanners.  This method has an effect on the point clouds that allows more information to  be  obtained,  especially  towards  a  higher  view,  howeverthe  scanning  times  will  be  longer  than  in  "Scan  and  Go" mode.

To start the reconstruction in this mode, the following python script is executed:

`python stop_and_scan_color.py` or `python stop_and_scan_no_color.py`

This scan mode requires the operator to move the robot manually in the directions that require rebuilding, therefore, the following command must be executed to tele-operate the robot.  

`roslaunch turtlebot_teleop keyboard_teleop.launch` or `roslaunch turtlebot_teleop ps3_teleop.launch`

At the points in the scenario where the reconstruction is required the pilot for the robot and sends a signal through a button generated by the python `send_command.py` script.  The robot starts the reconstruction and you must wait until the robot makes a sound, indicating that the reconstruction is finished and you must tele-operate the robot manually to the next point.  This operation is performed in the amount of points to be reconstructed from that environment.

When the path is finished, the environment map has been reconstructed, the text files are generated that save the X, Y, Z, R, G, B information of the environment map.  An example of the generated files is in the `data` folder. A 3D environment generated in this reconstruction mode is shown below.

![Image3](https://github.com/julianchaux/3D_XYZRGB_Reconstruction/blob/master/data/Room%20Color%20Test.jpg)

## Authors

-   Harold F. Murcia - ([www.haroldmurcia.com](http://www.haroldmurcia.com/))
-   Julián R. Cháux - ([jchaux@misena.educo](mailto:jchaux@misena.educo))

## License

This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or distribute this software, either in source code form or as a compiled binary, for any purpose, commercial or non-commercial, and by any means.

In jurisdictions that recognize copyright laws, the author or authors of this software dedicate any and all copyright interest in the software to the public domain. We make this dedication for the benefit of the public at large and to the detriment of our heirs and successors. We intend this dedication to be an overt act of relinquishment in perpetuity of all present and future rights to this software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to  [http://unlicense.org](http://unlicense.org/)
