## Updates on Moveit with Gripper/Vacuum Gripper:
&ensp;&ensp;Please pay attention if you are using Moveit motion planning for xArm models with Gripper/Vacuum Gripper attached, in updates since **Jan 6, 2021**, pose command/feedback will count in the **TCP offset** of the tool, as a result in moveit configuration change. 

2021년 1월 6일 이후 업데이트에서 Gripper/Vacuum Gripper가 부착된 xArm 모델에 대해 Moveit 모션 플래닝을 사용하는 경우, moveit 구성 변경의 결과로 포즈 명령/피드백이 도구 의 **TCP 오프셋** 에 계산됩니다 .

## Important Notice:
&ensp;&ensp;Due to robot communication data format change, ***early users*** (xArm shipped ***before June 2019***) are encouraged to ***upgrade*** their controller firmware immediately to drive the robot normally in future updates as well as to use newly developed functions. Please contact our staff to get instructions of the upgrade process. The old version robot driver can still be available in ***'legacy'*** branch, however, it will not be updated any more.   

&ensp;&ensp;You MUST follow **chapter 3** to install additional packages needed before any usage of xarm_ros packages. Otherwise, unexpected errors may occur.

&ensp;&ensp;If developing with **Moveit**, it is highly recommended to use **DIRECT network cable connection** between controller box and your PC, and no intermediate switches or routers, or the communication latency may have a bad impact on trajectory execution.  

  로봇 통신 데이터 형식 변경으로 인해 ***초기 사용자*** (xArm은 ***2019년 6월 이전에*** 배송됨 )는 컨트롤러 펌웨어를 즉시 ***업그레이드*** 하여 향후 업데이트에서 로봇을 정상적으로 구동하고 새로 개발된 기능을 사용하는 것이 좋습니다. 업그레이드 프로세스에 대한 지침을 얻으려면 직원에게 문의하십시오. 이전 버전의 로봇 드라이버는 ***'레거시'*** 분기 에서 계속 사용할 수 있지만 더 이상 업데이트되지 않습니다.

  당신은 반드시 지켜야 **3 장을** xarm_ros 패키지의 사용하기 전에 필요한 추가 패키지를 설치합니다. 그렇지 않으면 예기치 않은 오류가 발생할 수 있습니다.

**Moveit으로**  개발하는 경우 컨트롤러 박스와 PC 사이에 **DIRECT 네트워크 케이블 연결** 을 사용 하고 중간 스위치나 라우터를 사용하지 않는 것이 좋습니다 . 그렇지 않으면 통신 대기 시간이 궤적 실행에 나쁜 영향을 미칠 수 있습니다.


# Contents:  
* [1. Introduction](#1-introduction)
* [2. Update History](#2-update-summary)
* [3. Preparations (**MUST DO**)](#3-preparations-before-using-this-package)
* [4. Get Started](#4-getting-started-with-xarm_ros)
* [5. Package Description & Usage Guidance](#5-package-description--usage-guidance)
    * [5.1 xarm_description](#51-xarm_description)  
    * [5.2 xarm_gazebo](#52-xarm_gazebo)  
    * [5.3 xarm_controller](#53-xarm_controller)  
    * [5.4 xarm_bringup](#54-xarm_bringup)  
    * [5.5 ***xarm7_moveit_config***](#55-xarm7_moveit_config)  
        * [5.5.1 Add Custom Tool Model For Moveit](#551-add-custom-tool-model-for-moveit)  
    * [5.6 ***xarm_planner***](#56-xarm_planner)  
    * [5.7 ***xarm_api/xarm_msgs***](#57-xarm_apixarm_msgs)  
        * [5.7.1 Starting xArm by ROS service (***priority for the following operations***)](#starting-xarm-by-ros-service)  
        * [5.7.2 Joint space or Cartesian space command example(**Velocity Control** Added)](#joint-space-or-cartesian-space-command-example)
        * [5.7.3 Tool/Controller I/O Operations](#tool-io-operations)  
        * [5.7.4 Getting status feedback](#getting-status-feedback)  
        * [5.7.5 Setting Tool Center Point Offset](#setting-tool-center-point-offset)  
        * [5.7.6 Clearing Errors](#clearing-errors)  
        * [5.7.7 Gripper Control](#gripper-control)
        * [5.7.8 Vacuum Gripper Control](#vacuum-gripper-control)
        * [5.7.9 Tool Modbus communication](#tool-modbus-communication)
* [6. Mode Change](#6-mode-change)
    * [6.1 Mode Explanation](#61-mode-explanation)
    * [6.2 Proper way to change modes](#62-proper-way-to-change-modes)
* [7. Other Examples](#7-other-examples)
    * [7.1 Multi-xArm5 (separate control)](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#1-multi_xarm5-controlled-separately)
    * [7.2 Servo_Cartesian](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#2-servo_cartesian-streamed-cartesian-trajectory)
    * [7.3 Servo_Joint](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#3-servo_joint-streamed-joint-space-trajectory)
    * [7.4 Dual xArm6 controlled with one moveGroup node](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#4-dual-xarm6-controlled-with-one-movegroup-node)
    * [7.5 An example of demonstrating redundancy resolution using MoveIt](https://github.com/xArm-Developer/xarm_ros/tree/master/examples/xarm7_redundancy_res)

# 1. Introduction
   &ensp;&ensp;This repository contains the 3D models of xArm series and demo packages for ROS development and simulations.Developing and testing environment: Ubuntu 16.04 + ROS Kinetic/Melodic.  
   ***Instructions below is based on xArm7, other model user can replace 'xarm7' with 'xarm6' or 'xarm5' where applicable.***
   For simplified Chinese instructions: [简体中文版](./ReadMe_cn.md)    
   
     이 저장소에는 xArm 시리즈의 3D 모델과 ROS 개발 및 시뮬레이션을 위한 데모 패키지가 포함되어 있습니다. 개발 및 테스트 환경: Ubuntu 16.04 + ROS Kinetic/Melodic.
***아래 지침은 xArm7을 기준으로 하며, 다른 모델 사용자는 해당되는 경우 'xarm7'을 'xarm6' 또는 'xarm5'로 대체할 수 있습니다.***

# 2. Update Summary
   This package is still under development and improvement, tests, bug fixes and new functions are to be updated regularly in the future. 
   * Add xArm 7 description files, meshes and sample controller demos for ROS simulation and visualization.
   * Add Moveit! planner support to control Gazebo virtual model and real xArm, but the two can not launch together.
   * Add Direct control of real xArm through Moveit GUI, please use it with special care.
   * Add xArm hardware interface to use ROS position_controllers/JointTrajectoryController on real robot.
   * Add xArm 6 and xArm 5 simulation/real robot control support.
   * Add simulation model of xArm Gripper.
   * Add demo to control dual xArm6 through Moveit.
   * Add xArm Gripper action control.
   * Add xArm-with-gripper Moveit development packages.
   * Add vacuum gripper model and xArm-with-vacuum-gripper Moveit development packages (under /examples dir).
   * Thanks to [Microsoft IoT](https://github.com/ms-iot), xarm_ros can now be compiled and run on Windows platform.
   * Add velocity control mode for joint and Cartesian space. (**xArm controller firmware version >= 1.6.8** required)  
   * Add support for [custom tool model for Moveit](#551-add-custom-tool-model-for-moveit)  

# 3. Preparations before using this package

## 3.1 Install dependent package module 종속 패키지 모듈 설치
   gazebo_ros_pkgs: <http://gazebosim.org/tutorials?tut=ros_installing> (if use Gazebo)   
   ros_control: <http://wiki.ros.org/ros_control> (remember to select your correct ROS distribution)  
   moveit_core: <https://moveit.ros.org/install/>  
   
## 3.2 Go through the official tutorial documents 공식 튜토리얼 문서 살펴보기
ROS Wiki: <http://wiki.ros.org/>  
Gazebo Tutorial: <http://gazebosim.org/tutorials>  
Gazebo ROS Control: <http://gazebosim.org/tutorials/?tut=ros_control>  
Moveit tutorial: <http://docs.ros.org/kinetic/api/moveit_tutorials/html/>  

## 3.3 Download the 'table' 3D model Gazebo에서 xArm Gripper 시뮬레이션을 위한 "mimic_joint_plugin" 설치
&ensp;&ensp;In Gazebo simulator, navigate through the model database for 'table' item, drag and place the 3D model inside the virtual environment. It will then be downloaded locally, as 'table' is needed for running the demo.

  Gazebo 시뮬레이터에서 '테이블' 항목에 대한 모델 데이터베이스를 탐색하고 가상 환경 내부에 3D 모델을 끌어 놓습니다. 그런 다음 데모를 실행하는 데 '테이블'이 필요하므로 로컬로 다운로드됩니다.

## 3.4 Install "mimic_joint_plugin" for xArm Gripper simulation in Gazebo
&ensp;&ensp;If simulating xArm Gripper in Gazebo is needed, [**mimic_joint_plugin**](https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins) by courtesy of Konstantinos Chatzilygeroudis (@costashatz) needs to be installed in order to make the mimic joints behave normally in Gazebo. Usage of this plugin is inspired by [this tutorial](https://github.com/mintar/mimic_joint_gazebo_tutorial) from @mintar.   

12/22/2020: Refer to issue #53, Please Note this plugin has recently been **deprecated**, if you plan to use [new version](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins), please change "libroboticsgroup_gazebo_mimic_joint_plugin.so" to "libroboticsgroup_upatras_gazebo_mimic_joint_plugin.so" in file: xarm_ros/xarm_gripper/urdf/xarm_gripper.gazebo.xacro 

  Gazebo에서 xArm Gripper를 시뮬레이션 해야 하는 경우 Gazebo에서 모방 조인트가 정상적으로 작동하도록 하기 위해 Konstantinos Chatzilygeroudis(@costashatz)의 호의에 따라 [**mimic_joint_plugin**](https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins) 을 설치해야 합니다. 이 플러그인의 사용법은 @mintar의 [이 튜토리얼](https://github.com/mintar/mimic_joint_gazebo_tutorial) 에서 영감 을 받았습니다.

2020년 12월 22일: 문제 #53을 참조하십시오. 이 플러그인은 최근에 **더 이상 사용되지 않습니다** . [새 버전](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins) 을 사용하려는 경우 파일에서 "libroboticsgroup_gazebo_mimic_joint_plugin.so"를 "libroboticsgroup_upatras_gazebo_mimic_joint_plugin.so/"로 변경하십시오: xarm_farm /xarm_gripper.gazebo.xacro

# 4. Getting started with 'xarm_ros'
   
## 4.1 Create a catkin workspace. 
   &ensp;&ensp;If you already have a workspace, skip and move on to next part.
   Follow the instructions in [this page](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 
   Please note that this readme instruction assumes the user continues to use '~/catkin_ws' as directory of the workspace.
   

  이미 작업 공간이 있는 경우 건너뛰고 다음 부분으로 이동합니다. [이 페이지](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) 의 지침을 따르십시오 . 이 readme 지침은 사용자가 작업 공간의 디렉토리로 '~/catkin_ws'를 계속 사용한다고 가정합니다.



## 4.2 Obtain the package 패키지 받기
   ```bash
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/xArm-Developer/xarm_ros.git --recursive
   ```

## 4.2.1 update the package
   ```bash
   $ cd ~/catkin_ws/src/xarm_ros
   $ git pull
   $ git submodule sync
   $ git submodule update --init --remote
   ```

## 4.3 Install other dependent packages: 다른 종속 패키지 설치:
   ```bash
   $ rosdep update
   $ rosdep check --from-paths . --ignore-src --rosdistro kinetic
   ```
   Please change 'kinetic' to the ROS distribution you use. If there are any missing dependencies listed. Run the following command to install:  
   'kinetic'을 사용하는 ROS 분포로 변경하십시오. 누락된 종속성이 나열되는 경우. 다음 명령을 실행하여 설치합니다.
   
   ```bash
   $ rosdep install --from-paths . --ignore-src --rosdistro kinetic -y
   ```
   And chane 'kinetic' to the ROS distribution you use.  
   그리고 'kinetic'을 사용하는 ROS 분포로 변경하십시오.

## 4.4 Build the code
   ```bash
   $ cd ~/catkin_ws
   $ catkin_make
   ```
## 4.5 Source the setup script
```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
Skip above operation if you already have that inside your ~/.bashrc. Then do:
```bash
$ source ~/.bashrc
```
## 4.6 First try out in RViz:
```bash
$ roslaunch xarm_description xarm6_rviz_display.launch
```

## 4.7 Run the demo in Gazebo simulator
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch [run_demo:=true] [add_gripper:=true] [add_vacuum_gripper:=true]
   
   $ roslaunch xarm_gazebo xarm7_beside_table.launch run_demo:=true add_gripper:=true

   ```
&ensp;&ensp;Add the "run_demo" option if you wish to see a pre-programed loop motion in action. The command trajectory is written in xarm_controller\src\sample_motion.cpp. And the trajectory in this demo is controlled by pure position interface.   
&ensp;&ensp;Add the "add_gripper" option if you want to see the xArm Gripper attached at the tool end.  
&ensp;&ensp;Add the "add_vacuum_gripper" option if you want to see the xArm Vacuum Gripper attached at the tool end. Please note ONLY ONE end effector can be attached.  

  사전 프로그래밍된 루프 동작이 실행되는 것을 보려면 "run_demo" 옵션을 추가하십시오. 명령 궤적은 xarm_controller\src\sample_motion.cpp에 기록됩니다. 그리고 이 데모의 궤적은 순수 위치 인터페이스에 의해 제어됩니다.
  도구 끝에 부착된 xArm 그리퍼를 보려면 "add_gripper" 옵션을 추가하십시오.
  xArm 진공 그리퍼가 공구 끝에 부착된 것을 보려면 "add_vacuum_gripper" 옵션을 추가하십시오. 단 하나의 엔드 이펙터만 부착할 수 있습니다.

# 5. Package description & Usage Guidance
   
## 5.1 xarm_description
   &ensp;&ensp;xArm description files, mesh files and gazebo plugin configurations, etc. It's not recommended to change the xarm description file since other packages depend on it. 
   
  xArm 설명 파일, 메쉬 파일 및 가제보 플러그인 구성 등. xarm 설명 파일은 다른 패키지가 의존하므로 변경하지 않는 것이 좋습니다.

## 5.2 xarm_gazebo
   &ensp;&ensp;Gazebo world description files and simulation launch files. User can add or build their own models in the simulation world file.
   
     Gazebo 세계 설명 파일 및 시뮬레이션 실행 파일. 사용자는 시뮬레이션 세계 파일에 자신의 모델을 추가하거나 구축할 수 있습니다.

## 5.3 xarm_controller
   &ensp;&ensp;Controller configurations, hardware_interface, robot command executable source, scripts and launch files. User can deploy their program inside this package or create their own. ***Note that*** effort controllers defined in xarm_controller/config are just examples for simulation purpose, when controlling the real arm, only 'position_controllers/JointTrajectoryController' interface is provided. User can add their self-defined controllers as well, refer to: http://wiki.ros.org/ros_control (controllers)
   
     컨트롤러 구성, hardware_interface, 로봇 명령 실행 가능 소스, 스크립트 및 실행 파일. 사용자는 이 패키지 안에 프로그램을 배포하거나 직접 만들 수 있습니다. ***참고*** 노력 컨트롤러가 실제 암을 제어 할 때 xarm_controller / 설정이 시뮬레이션 목적을 위해 단지 예, 오직 'position_controllers이 / JointTrajectoryController'인터페이스가 제공하는 정의. 사용자는 자체 정의 컨트롤러도 추가할 수 있습니다. http://wiki.ros.org/ros_control (컨트롤러)를 참조하십시오.

## 5.4 xarm_bringup  
&ensp;&ensp;launch files to load xarm driver to enable direct control of real xArm hardware.  

  파일을 실행하여 xarm 드라이버를 로드하여 실제 xArm 하드웨어를 직접 제어할 수 있습니다.

## 5.5 xarm7_moveit_config
Please note: xarm_moveit_config related packages will limit all joints within `[-pi, pi]`, it seems that moveit tend to generate plans involving greater joint motions if not limited within this range. This limit can be canceled by setting "limited:=false" in `...moveit_config/launch/planning_context.launch`.

참고: xarm_moveit_config 관련 패키지는 내 모든 관절을 제한합니다 `[-pi, pi]`. 이 범위 내에서 제한되지 않는 경우 moveit 은 더 큰 관절 동작을 포함하는 계획을 생성하는 경향이 있는 것 같습니다. 이 제한은 에서 "limited:=false"를 설정하여 취소할 수 있습니다 `...moveit_config/launch/planning_context.launch`.

&ensp;&ensp;This package is partially generated by moveit_setup_assistant, could use with Moveit Planner and Rviz visualization. If you have Moveit! installed, you can try the demo.

  이 패키지는 moveit_setup_assistant에 의해 부분적으로 생성되며 Moveit Planner 및 Rviz 시각화와 함께 사용할 수 있습니다. 무브잇이 있다면! 설치하면 데모를 시도할 수 있습니다.
  
   ```bash
   $ roslaunch xarm7_moveit_config demo.launch
   ```
#### To run Moveit! motion planner along with Gazebo simulator:  
   1. If no xArm gripper needed, first run:  
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch
   ```
   Then in another terminal:
   ```bash
   $ roslaunch xarm7_moveit_config xarm7_moveit_gazebo.launch
   ```
   2. If **xArm gripper needs to be attached**, first run:  
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch add_gripper:=true
   ```
   Then in another terminal:
   ```bash
   $ roslaunch xarm7_gripper_moveit_config xarm7_gripper_moveit_gazebo.launch
   ```
   If you have a satisfied motion planned in Moveit!, hit the "Execute" button and the virtual arm in Gazebo will execute the trajectory.   Moveit!에서 계획된 만족스러운 모션이 있는 경우 "실행" 버튼을 누르면 Gazebo의 가상 팔이 궤적을 실행합니다.

   3. If **xArm vacuum gripper needs to be attached**, just replace "gripper" with "vacuum_gripper" in above gripper example.  경우 **xArm 진공 그립퍼가 부착 될 필요가** 단지 상기 그리퍼 예에서 "vacuum_gripper"와 "그리퍼"대체.

#### To run Moveit! motion planner to control the real xArm:  
   First make sure the xArm and the controller box are powered on, then execute:  
   
   ```bash
   $ roslaunch xarm7_moveit_config realMove_exec.launch robot_ip:=<your controller box LAN IP address>
   ```
   Examine the terminal output and see if any error occured during the launch. If not, just play with the robot in Rviz and you can execute the sucessfully planned trajectory on real arm. But be sure it will not hit any surroundings before execution!  

#### To run Moveit! motion planner to control the real xArm with xArm Gripper attached:  Moveit을 실행하려면! xArm 그리퍼가 부착된 실제 xArm을 제어하는 모션 플래너:
   First make sure the xArm and the controller box are powered on, then execute:  먼저 xArm과 컨트롤러 상자의 전원이 켜져 있는지 확인한 다음 다음을 실행합니다.
   ```bash
   $ roslaunch xarm7_gripper_moveit_config realMove_exec.launch robot_ip:=<your controller box LAN IP address>
   ```
   It is better to use this package with real xArm gripper, since Moveit planner will take the gripper into account for collision detection.  
   Moveit 플래너는 충돌 감지를 위해 그리퍼를 고려하므로 실제 xArm 그리퍼와 함께 이 패키지를 사용하는 것이 좋습니다

#### To run Moveit! motion planner to control the real xArm with xArm Vacuum Gripper attached:  
   First make sure the xArm and the controller box are powered on, then execute:  
   ```bash
   $ roslaunch xarm7_vacuum_gripper_moveit_config realMove_exec.launch robot_ip:=<your controller box LAN IP address>
   ```
   It is better to use this package with real xArm vacuum gripper, since Moveit planner will take the vacuum gripper into account for collision detection.  

## 5.5.1 Add custom tool model for Moveit Moveit용 커스텀 도구 모델 추가
&ensp;&ensp;***This part may require ROS Melodic or later versions to function well***  
&ensp;&ensp;For __xarm5_moveit_config__/__xarm6_moveit_config__/__xarm7_moveit_config__, customized tool models maybe added to the tool flange through quick-configuration parameters listed below，thus to enable Tool offset and 3D collision checking during Moveit motion planning. (Notice：configuration through '/xarm/set_tcp_offset' service will not be effective in Moveit planning!) 

  ***이 부분은 ROS 멜로디 나 잘 기능에 이상 버전이 필요할 수 있습니다***
  들어를 **xarm5_moveit_config** / **xarm6_moveit_config** / **xarm7_moveit_config** , 어쩌면 따라서 공구 옵셋 활성화하고 Moveit 운동 계획 중 3D 충돌 검사하기 위해, 아래의 빠른 구성 매개 변수를 통해 도구 플랜지에 추가 된 사용자 정의 도구 모델. (참고: '/xarm/set_tcp_offset' 서비스를 통한 구성은 Moveit 계획에서 유효하지 않습니다!)
  

### Examples:
   ```bash
   # attaching box model:
   $ roslaunch xarm7_moveit_config demo.launch add_other_geometry:=true geometry_type:=box

   # attaching cylinder model:
   $ roslaunch xarm7_moveit_config demo.launch add_other_geometry:=true geometry_type:=cylinder

   # attaching sphere model:
   $ roslaunch xarm7_moveit_config demo.launch add_other_geometry:=true geometry_type:=sphere

   # attaching customized mesh model:（Here take xarm vacuum_gripper as an example，if the mesh model could be placed in: 'xarm_description/meshes/other'directory，'geometry_mesh_filename' argument can be simplified to be just the filename）
   $ roslaunch xarm7_moveit_config demo.launch add_other_geometry:=true geometry_type:=mesh geometry_mesh_filename:=package://xarm_description/meshes/vacuum_gripper/visual/vacuum_gripper.STL geometry_mesh_tcp_xyz:='"0 0 0.126"'
   ```

### Argument explanations:
- __add_other_geometry__: default to be false，indicating whether to add other geometry model to the tool.
- __geometry_type__: geometry shapes to be added，as one of 'box/cylinder/sphere/mesh', there are different parameters required for different types.  
- __geometry_height__: height of geometry shape，unit: meter，default value: 0.1，effective for geometry_type: box/cylinder/sphere.
- __geometry_radius__: radius of geometry shape，unit: meter，default value: 0.1，effective for geometry_type: cylinder/sphere.
- __geometry_length__: length of geometry shape，unit: meter，default value: 0.1，effective for geometry_type: box.
- __geometry_width__: width of geometry shape，unit: meter，default value: 0.1，effective for geometry_type: box.
- __geometry_mesh_filename__: geometry shape，effective for geometry_type: mesh.
- __geometry_mesh_origin_xyz__: position offset from mesh base coordinate to xarm tool-flange coordinate, default: "0 0 0"，effective for geometry_type: mesh.
- __geometry_mesh_origin_rpy__: orientation offset from mesh base coordinate to xarm tool-flange coordinate, default: "0 0 0"，effective for geometry_type: mesh.
- __geometry_mesh_tcp_xyz__: the positional TCP offset with respect to xarm tool-flange coordinate, default: "0 0 0"，effective for geometry_type: mesh.
- __geometry_mesh_tcp_rpy__: the orientational TCP offset with respect to xarm tool-flange coordinate, default: "0 0 0"，effective for geometry_type: mesh.  

### 인수 설명:

- **add_other_geometry** : 기본값은 false로 도구에 다른 지오메트리 모델을 추가할지 여부를 나타냅니다.
- **geometry_type** : 추가할 기하학 모양, 'box/cylinder/sphere/mesh' 중 하나로 유형에 따라 다른 매개변수가 필요합니다.
- **geometry_height** : **기하 도형의** 높이, 단위: 미터, 기본값: 0.1, geometry_type: 상자/실린더/구에 유효합니다.
- **geometry_radius** : 기하학 모양의 반지름, 단위: 미터, 기본값: 0.1, geometry_type에 유효: 실린더/구.
- **geometry_length** : **기하 도형의** 길이, 단위: 미터, 기본값: 0.1, geometry_type: box에 유효합니다.
- **geometry_width** : **기하 도형의** 너비, 단위: 미터, 기본값: 0.1, geometry_type: 상자에 유효합니다.
- **geometry_mesh_filename** : 기하 도형, geometry_type: mesh에 유효합니다.
- **geometry_mesh_origin_xyz** : 메쉬 기본 좌표에서 xarm 도구 플랜지 좌표까지의 위치 오프셋, 기본값: "0 0 0", geometry_type: 메쉬에 유효합니다.
- **geometry_mesh_origin_rpy** : 메쉬 기본 좌표에서 xarm 도구 플랜지 좌표까지의 방향 오프셋, 기본값: "0 0 0", geometry_type: 메쉬에 유효합니다.
- **geometry_mesh_tcp_xyz** : xarm 도구 플랜지 좌표에 대한 위치 TCP 오프셋, 기본값: "0 0 0", geometry_type: mesh에 유효합니다.
- **geometry_mesh_tcp_rpy** : xarm 도구 플랜지 좌표에 대한 방향 TCP 오프셋, 기본값: "0 0 0", geometry_type: mesh에 유효합니다.


## 5.6 xarm_planner:
&ensp;&ensp;This implemented simple planner interface is based on move_group from Moveit! and provide ros service for users to do planning & execution based on the requested target, user can find detailed instructions on how to use it inside [***xarm_planner package***](./xarm_planner/).  

  이 구현된 간단한 플래너 인터페이스는 Moveit!의 move_group을 기반으로 합니다! 사용자가 요청한 대상을 기반으로 계획 및 실행을 수행할 수 있도록 ros 서비스를 제공합니다. 사용자는 [***xarm_planner 패키지***](https://github.com/HyeWon33/xarm_ros/blob/master/xarm_planner) 내에서 사용 방법에 대한 자세한 지침을 찾을 수 있습니다 .
  
#### To launch the xarm simple motion planner together with the real xArm: 실제 xArm과 함께 xarm 심플 모션 플래너를 시작하려면:

  
```bash
   $ roslaunch xarm_planner xarm_planner_realHW.launch robot_ip:=<your controller box LAN IP address> robot_dof:=<7|6|5> add_(vacuum_)gripper:=<true|false>
```
Argument 'robot_dof' specifies the number of joints of your xArm (default is 7). Now xarm_planner supports model with gripper or vacuum_gripper attached. Please specify "**add_gripper**" or "**add_vacuum_gripper**" argument if needed.    

'robot_dof' 인수는 xArm의 관절 수를 지정합니다(기본값은 7). 이제 xarm_planner는 그리퍼 또는 vacuum_gripper가 부착된 모델을 지원합니다. 필요한 경우 " **add_gripper** " 또는 " **add_vacuum_gripper** " 인수를 지정하십시오 .



## 5.7 xarm_api/xarm_msgs:
&ensp;&ensp;These two packages provide user with the ros service wrapper of the functions in xArm SDK. There are 8 types of motion command (service names) supported，please set correct robot mode first, refer to [mode change section](#6-mode-change):  
#### Robot Mode 0:
* <font color=blue>[move_joint](#1-joint-space-motion):</font> joint space point to point command, given target joint angles, max joint velocity and acceleration. Corresponding function in SDK is "set_servo_angle()".  
* <font color=blue>[move_line](#2-cartesian-space-motion-in-base-coordinate):</font> straight-line motion to the specified Cartesian Tool Centre Point(TCP) target. Corresponding function in SDK is "set_position()"[blending radius not specified].  
* <font color=blue>move_lineb:</font> a list of via points followed by target Cartesian point. Each segment is straight-line with Arc blending at the via points, to make velocity continuous. Corresponding function in SDK is "set_position()"[blending radius specified]. Please refer to [move_test.cpp](./xarm_api/test/move_test.cpp) for example code.   
* <font color=blue>[move_line_tool](#3-cartesian-space-motion-in-tool-coordinate):</font> straight-line motion based on the **Tool coordinate system** rather than the base system. Corresponding function in SDK is "set_tool_position()".  
Please ***keep in mind that*** before calling the 4 motion services above, first set robot mode to be 0, then set robot state to be 0, by calling relavent services. Meaning of the commands are consistent with the descriptions in product ***user manual***, other xarm API supported functions are also available as service call. Refer to [xarm_msgs package](./xarm_msgs/) for more details and usage guidance.  

* <font color=blue>[move_line_aa](#4-cartesian-space-motion-in-axis-angle-orientation):</font> straight-line motion, with orientation expressed in **Axis-angle** rather than roll-pitch-yaw angles. Please refer to xArm user manual for detailed explanation of axis-angle before using this command.   

#### Robot Mode 1:
* <font color=blue>[move_servo_cart](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#2-servo_cartesian-streamed-cartesian-trajectory)/[move_servoj](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#3-servo_joint-streamed-joint-space-trajectory):</font> streamed high-frequency trajectory command execution in Cartesian space or joint space. Corresponding functions in SDK are set_servo_cartesian() and set_servo_angle_j(). An alternative way to implement <font color=red>velocity control</font>. Special **RISK ASSESMENT** is required before using them. Please read the guidance carefully at [chapter 7.2-7.3](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#2-servo_cartesian-streamed-cartesian-trajectory).  

#### Robot Mode 4:
* <font color=blue>[velo_move_joint](#5-joint-velocity-control):</font> Joint motion with specified velocity for each joint (unit: rad/s), with maximum joint acceleration configurable by `set_max_acc_joint` service.  

#### Robot Mode 5:
* <font color=blue>[velo_move_line](#6-cartesian-velocity-control):</font> Linear motion of TCP with specified velocity in mm/s (position) and rad/s (orientation in **axis-angular_velocity**), with maximum linear acceleration configurable by `set_max_acc_line` service. 

##  xarm_api/xarm_msgs:

  이 두 패키지는 사용자에게 xArm SDK의 기능에 대한 ros 서비스 래퍼를 제공합니다. 지원되는 모션 명령(서비스 이름)에는 8가지 유형이 있습니다. 먼저 올바른 로봇 모드를 설정하십시오. [모드 변경 섹션을](https://github.com/HyeWon33/xarm_ros#6-mode-change) 참조하십시오 .

#### 로봇 모드 0:

- [move_joint](https://github.com/HyeWon33/xarm_ros#1-joint-space-motion) : 주어진 목표 관절 각도, 최대 관절 속도 및 가속도를 기준으로 관절 공간 점 대 점 명령. SDK의 해당 함수는 "set_servo_angle()"입니다.
- [move_line](https://github.com/HyeWon33/xarm_ros#2-cartesian-space-motion-in-base-coordinate) : 지정된 Cartesian Tool Center Point(TCP) 대상에 대한 직선 모션. SDK의 해당 기능은 "set_position()"[혼합 반경이 지정되지 않음]입니다.
- move_lineb: 대상 데카르트 포인트가 뒤따르는 경유 포인트 목록. 각 세그먼트는 속도를 연속적으로 만들기 위해 비아 포인트에서 Arc 혼합으로 직선입니다. SDK의 해당 기능은 "set_position()"[혼합 반경 지정]입니다. 예제 코드 는 [move_test.cpp](https://github.com/HyeWon33/xarm_ros/blob/master/xarm_api/test/move_test.cpp) 를 참조하십시오 .
- [move_line_tool](https://github.com/HyeWon33/xarm_ros#3-cartesian-space-motion-in-tool-coordinate) : 기본 시스템이 아닌 **도구 좌표 시스템을** 기반으로 하는 직선 모션 . SDK의 해당 함수는 "set_tool_position()"입니다.
  제발 ***것을 명심*** 위의 4 개 모션 서비스를 호출하기 전에 먼저 설정 로봇 모드, 0으로 다음 설정 로봇의 상태가 relavent 서비스를 호출하여, 0이 될 수 있습니다. 명령의 의미는 제품 ***사용 설명서*** 의 설명과 일치하며 다른 xarm API 지원 기능도 서비스 호출로 사용할 수 있습니다. 자세한 내용 및 사용 지침 은 [xarm_msgs 패키지](https://github.com/HyeWon33/xarm_ros/blob/master/xarm_msgs) 를 참조하세요.
- [move_line_aa](https://github.com/HyeWon33/xarm_ros#4-cartesian-space-motion-in-axis-angle-orientation) : 직선 모션, 방향은 롤-피치-요 각도가 아닌 **축 각도로** 표시됩니다 . 이 명령을 사용하기 전에 축 각도에 대한 자세한 설명은 xArm 사용 설명서를 참조하십시오.

#### 로봇 모드 1:

- [move_servo_cart](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#2-servo_cartesian-streamed-cartesian-trajectory) / [move_servoj](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#3-servo_joint-streamed-joint-space-trajectory) : 데카르트 공간 또는 조인트 공간에서 스트리밍된 고주파 궤적 명령 실행. SDK의 해당 함수는 set_servo_cartesian() 및 set_servo_angle_j()입니다. 속도 제어를 구현하는 다른 방법입니다. 사용하기 전에 특별 **위험 평가** 가 필요합니다. [7.2-7.3장의](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#2-servo_cartesian-streamed-cartesian-trajectory) 지침을 주의 깊게 읽으십시오 .

#### 로봇 모드 4:

- [velo_move_joint](https://github.com/HyeWon33/xarm_ros#5-joint-velocity-control) : 각 관절에 대해 지정된 속도로 관절 모션(단위: rad/s), 최대 관절 가속도는 `set_max_acc_joint`서비스 별로 구성할 수 있습니다.

#### 로봇 모드 5:

- [velo_move_line](https://github.com/HyeWon33/xarm_ros#6-cartesian-velocity-control) : 서비스에 의해 구성 가능한 최대 선형 가속과 함께 mm/s(위치) 및 rad/s( **axis-angular_velocity의** 방향) 단위로 지정된 속도를 갖는 TCP의 선형 동작 `set_max_acc_line`.




#### Starting xArm by ROS service: ROS 서비스로 xArm 시작:

&ensp;&ensp;First startup the service server for xarm7, ip address is just an example:  먼저 xarm7용 서비스 서버를 시작합니다. ip 주소는 예시일 뿐입니다.
```bash
$ roslaunch xarm_bringup xarm7_server.launch robot_ip:=192.168.1.128
```
&ensp;&ensp;Then make sure all the servo motors are enabled, refer to [SetAxis.srv](/xarm_msgs/srv/SetAxis.srv): 그런 다음 모든 서보 모터가 활성화되었는지 확인하십시오. [SetAxis.srv를](https://github.com/HyeWon33/xarm_ros/blob/master/xarm_msgs/srv/SetAxis.srv) 참조하십시오 .
```bash
$ rosservice call /xarm/motion_ctrl 8 1
```
&ensp;&ensp;Before any motion commands, set proper robot mode(0: POSE) and state(0: READY) ***in order***, refer to [SetInt16.srv](/xarm_msgs/srv/SetInt16.srv):   
모션 명령 전에 적절한 로봇 모드(0: POSE)와 상태(0: READY) ***를 순서대로 설정*** 하고 [SetInt16.srv를](https://github.com/HyeWon33/xarm_ros/blob/master/xarm_msgs/srv/SetInt16.srv) 참조하십시오 .  
```bash
$ rosservice call /xarm/set_mode 0

$ rosservice call /xarm/set_state 0
```

#### Joint space or Cartesian space command example: 조인트 공간 또는 데카르트 공간 명령 예:
&ensp;&ensp;Please note that all the angles must use the unit of ***radian***. All motion commands use the same type of srv request: [Move.srv](./xarm_msgs/srv/Move.srv).   
모든 각도는 ***라디안*** 단위를 사용해야 합니다 . 모든 모션 명령은 동일한 유형의 srv 요청인 [Move.srv를 사용](https://github.com/HyeWon33/xarm_ros/blob/master/xarm_msgs/srv/Move.srv) 합니다.

##### 1. Joint space motion:
&ensp;&ensp;To call joint space motion with max speed 0.35 rad/s and acceleration 7 rad/s^2: 최대 속도 0.35 rad/s 및 가속도 7 rad/s^2로 관절 공간 운동을 호출하려면:   
```bash
$ rosservice call /xarm/move_joint [0,0,0,0,0,0,0] 0.35 7 0 0
```
&ensp;&ensp;To go back to home (all joints at 0 rad) position with max speed 0.35 rad/s and acceleration 7 rad/s^2: 최대 속도 0.35 rad/s 및 가속도 7 rad/s^2로 홈(0 rad의 모든 관절) 위치로 돌아가려면:
```bash
$ rosservice call /xarm/go_home [] 0.35 7 0 0
```
##### 2. Cartesian space motion in Base coordinate: 도구 좌표에서의 직교 공간 운동:
&ensp;&ensp;To call Cartesian motion to the target expressed in robot BASE Coordinate, with max speed 200 mm/s and acceleration 2000 mm/s^2:
```bash
$ rosservice call /xarm/move_line [250,100,300,3.14,0,0] 200 2000 0 0
```
##### 3. Cartesian space motion in Tool coordinate:
&ensp;&ensp;To call Cartesian motion expressed in robot TOOL Coordinate, with max speed 200 mm/s and acceleration 2000 mm/s^2, the following will move a **relative motion** (delta_x=50mm, delta_y=100mm, delta_z=100mm) along the current Tool coordinate, no orientation change:

 로봇 TOOL 좌표로 표현된 직교 모션을 호출하기 위해 최대 속도 200mm/s, 가속도 2000mm/s^2, 다음은 현재 도구를 따라 **상대 모션** (delta_x=50mm, delta_y=100mm, delta_z=100mm)을 이동합니다. 좌표, 방향 변경 없음:
 
```bash
$ rosservice call /xarm/move_line_tool [50,100,100,0,0,0] 200 2000 0 0
```
##### 4. Cartesian space motion in Axis-angle orientation: 축 각도 방향의 데카르트 공간 운동:	
&ensp;&ensp;Corresponding service for Axis-angle motion is [MoveAxisAngle.srv](./xarm_msgs/srv/MoveAxisAngle.srv). Please pay attention to the last two arguments: "**coord**" is 0 for motion with respect to (w.r.t.) Arm base coordinate system, and 1 for motion w.r.t. Tool coordinate system. "**relative**" is 0 for absolute target position w.r.t. specified coordinate system, and 1 for relative target position.  

축 각도 모션에 해당하는 서비스는 [MoveAxisAngle.srv](https://github.com/HyeWon33/xarm_ros/blob/master/xarm_msgs/srv/MoveAxisAngle.srv) 입니다. 마지막 두 인수에 주의하십시오. " **coord** "는 (wrt) Arm 기본 좌표계에 대한 모션의 경우 0이고 모션 wrt 도구 좌표계의 경우 1입니다. " **relative** "는 지정된 좌표계의 절대 목표 위치에 대해 0이고 상대 목표 위치에 대해 1입니다.

&ensp;&ensp;For example: to move 1.0 radian relatively around tool-frame Z-axis: 
예: 도구 프레임 Z축을 기준으로 상대적으로 1.0 라디안을 이동하려면 다음을 수행합니다.

```bash
$ rosservice call /xarm/move_line_aa "pose: [0, 0, 0, 0, 0, 1.0]
mvvelo: 30.0
mvacc: 100.0
mvtime: 0.0
coord: 1
relative: 1" 
ret: 0
message: "move_line_aa, ret = 0"
```
Or
```bash
$ rosservice call /xarm/move_line_aa [0,0,0,0,0,1.0] 30.0 100.0 0.0 1 1
```   
&ensp;&ensp;"**mvtime**" is not meaningful in this command, just set it to 0. Another example: in base-frame, to move 122mm relatively along Y-axis, and rotate around X-axis for -0.5 radians:  

" **mvtime** "은 이 명령에서 의미가 없으며 그냥 0으로 설정합니다. 다른 예: 베이스 프레임에서 Y축을 따라 상대적으로 122mm 이동하고 -0.5 라디안 동안 X축을 중심으로 회전합니다.

```bash
$ rosservice call /xarm/move_line_aa [0,122,0,-0.5,0,0] 30.0 100.0 0.0 0 1  
```

##### 5. Joint velocity control: Joint velocity control: 관절 속도 제어:
&ensp;&ensp;(**xArm controller firmware version >= 1.6.8** required) If controlling joint velocity is desired, first switch to **Mode 4** as descriped in [mode change section](#6-mode-change). Please check the [MoveVelo.srv](./xarm_msgs/srv/MoveVelo.srv) first to understand the meanings of parameters reqired. If more than one joint are to move, set **jnt_sync** to 1 for synchronized acceleration/deceleration for all joints in motion, and if jnt_sync is 0, each joint will reach to its target velocity as fast as possible. ***coord*** parameter is not used here, just set it to 0. For example: 

( **xArm 컨트롤러 펌웨어 버전 >= 1.6.8** 필요) 관절 속도를 제어하려면 먼저 [모드 변경 섹션에 설명된](https://github.com/HyeWon33/xarm_ros#6-mode-change) 대로 **모드 4** 로 [전환하십시오](https://github.com/HyeWon33/xarm_ros#6-mode-change) . 필요한 매개변수의 의미를 이해하려면 먼저 [MoveVelo.srv를](https://github.com/HyeWon33/xarm_ros/blob/master/xarm_msgs/srv/MoveVelo.srv) 확인하십시오 . 둘 이상의 관절을 이동 해야 하는 경우 모션의 모든 관절에 대해 동기화된 가감속을 위해 **jnt_sync** 를 1로 설정 하고, jnt_sync가 0이면 각 관절이 최대한 빨리 목표 속도에 도달합니다. ***coord*** 매개변수는 여기에서 사용되지 않습니다. 그냥 0으로 설정하십시오. 예를 들면:

```bash
$ rosservice call /xarm/velo_move_joint [0.1,-0.1,0,0,0,-0.3] 1 0
``` 
will command the joints (for xArm6) to move in specified angular velocities (in rad/s) and they will reach to target velocities synchronously. The maximum joint acceleration can also be configured by (unit: rad/s^2):  

관절(xArm6용)이 지정된 각속도(rad/s)로 움직이도록 명령하고 동기적으로 목표 속도에 도달합니다. 최대 관절 가속도는 (단위: rad/s^2)로 구성할 수도 있습니다.

```bash
$ rosservice call /xarm/set_max_acc_joint 10.0  (maximum: 20.0 rad/s^2)
``` 

##### 6. Cartesian velocity control:  Cartesian velocity control: 직교 속도 제어:
&ensp;&ensp;(**xArm controller firmware version >= 1.6.8** required) If controlling linar velocity of TCP towards certain direction is desired, first switch to **Mode 5** as descriped in [mode change section](#6-mode-change). Please check the [MoveVelo.srv](./xarm_msgs/srv/MoveVelo.srv) first to understand the meanings of parameters reqired. Set **coord** to 0 for motion in world/base coordinate system and 1 for tool coordinate system. ***jnt_sync*** parameter is not used here, just set it to 0. For example: 

( **xArm 컨트롤러 펌웨어 버전 >= 1.6.8** 필요) 특정 방향으로 TCP의 선형 속도를 제어하려면 먼저 [모드 변경 섹션에 설명된](https://github.com/HyeWon33/xarm_ros#6-mode-change) 대로 **모드 5** 로 전환합니다 . 필요한 매개변수의 의미를 이해하려면 먼저 [MoveVelo.srv를](https://github.com/HyeWon33/xarm_ros/blob/master/xarm_msgs/srv/MoveVelo.srv) 확인하십시오 . 표준 /기준 좌표계의 모션에 대해 **좌표** 를 0으로 설정 하고 도구 좌표계에 대해 1로 설정합니다. ***jnt_sync*** 매개변수는 여기에서 사용되지 않고 0으로 설정하면 됩니다. 예를 들면 다음과 같습니다.

```bash
$ rosservice call /xarm/velo_move_line [30,0,0,0,0,0] 0 1
``` 
will command xArm TCP move along X-axis of TOOL coordinate system with speed of 30 mm/s. The maximum linear acceleration can also be configured by (unit: mm/s^2):  

xArm TCP는 TOOL 좌표계의 X축을 따라 30mm/s의 속도로 이동하도록 명령합니다. 최대 선형 가속도는 (단위: mm/s^2)로 구성할 수도 있습니다.

```bash
$ rosservice call /xarm/set_max_acc_line 5000.0  (maximum: 50000 mm/s^2)
``` 

For angular motion in orientation, please note the velocity is specified as **axis-angular_velocity** elements. That is, [the unit rotation axis vector] multiplied by [rotation velocity value(scalar)]. For example, 

방향 각운동의 경우 속도는 **axis-angular_velocity** 요소 로 지정됩니다 . 즉, [단위 회전축 벡터]에 [회전 속도 값(스칼라)]을 곱한 값입니다. 예를 들어,

```bash
$ rosservice call /xarm/velo_move_line [0,0,0,0.707,0,0] 0 0
``` 
This will command TCP to rotate along X-axis in BASE coordinates at about 45 degrees/sec. The maximum acceleration for orientation change is fixed.  

이것은 TCP가 약 45도/초로 BASE 좌표에서 X축을 따라 회전하도록 명령합니다. 방향 변경을 위한 최대 가속은 고정되어 있습니다.

Please Note: velocity motion can be stopped by either giving **all 0 velocity** command, or setting **state to 4(STOP)** and 0(READY) later for next motion.  
참고: 속도 모션은 **모두 0 속도** 명령을 주거나 다음 모션을 위해 나중에 **상태를 4(STOP)** 및 0(READY)으로 설정 하여 중지할 수 있습니다 . 

#### Motion service Return:
&ensp;&ensp;Please Note the above motion services will **return immediately** by default. If you wish to return until actual motion is finished, set the ros parameter **"/xarm/wait_for_finish"** to be **true** in advance. That is:  

위의 모션 서비스는 기본적으로 **즉시 반환** 됩니다. 실제 모션이 끝날 때까지 돌아가고 싶다면 미리 ros 매개변수 **"/xarm/wait_for_finish"** 를 **true** 로 설정하세요. 그건:

```bash
$ rosparam set /xarm/wait_for_finish true
```   
&ensp;&ensp;Upon success, 0 will be returned. If any error occurs, 1 will be returned.

#### Tool I/O Operations:

&ensp;&ensp;We provide 2 digital, 2 analog input port and 2 digital output signals at the end I/O connector.  
##### 1. To get current 2 DIGITAL input states:  
```bash
$ rosservice call /xarm/get_digital_in
```
##### 2. To get one of the ANALOG input value: 
```bash
$ rosservice call /xarm/get_analog_in 1  (last argument: port number, can only be 1 or 2)
```
##### 3. To set one of the Digital output:
```bash
$ rosservice call /xarm/set_digital_out 2 1  (Setting output 2 to be 1)
```
&ensp;&ensp;You have to make sure the operation is successful by checking responding "ret" to be 0.

#### Controller I/O Operations:

&ensp;&ensp;We provide 8/16 digital input and 8/16 digital output ports at controller box for general usage.  

##### 1. To get one of the controller DIGITAL input state:  
```bash
$ rosservice call /xarm/get_controller_din io_num (Notice: from 1 to 8, for CI0~CI7; from 9 to 16, for DI0~DI7[if any])  
```
##### 2. To set one of the controller DIGITAL output:
```bash
$ rosservice call /xarm/set_controller_dout io_num (Notice: from 1 to 8, for CO0~CO7; from 9 to 16, for DI0~DI7[if any]) logic (0 or 1) 
```
&ensp;&ensp;For example:  
```bash
$ rosservice call /xarm/set_controller_dout 5 1  (Setting output 5 [lable C04] to be 1)
```
##### 3. To get one of the controller ANALOG input:
```bash
$ rosservice call /xarm/get_controller_ain port_num  (Notice: from 1 to 2, for AI0~AI1)
```
##### 4. To set one of the controller ANALOG output:
```bash
$ rosservice call /xarm/set_controller_aout port_num (Notice: from 1 to 2, for AO0~AO1) analog_value
```
&ensp;&ensp;For example:  
```bash
$ rosservice call /xarm/set_controller_aout 2 3.3  (Setting port AO1 to be 3.3)
```
&ensp;&ensp;You have to make sure the operation is successful by checking responding "ret" to be 0.

#### Getting status feedback:
&ensp;&ensp;Having connected with a real xArm robot by running 'xarm7_server.launch', user can subscribe to the topic ***"xarm/xarm_states"*** for feedback information about current robot states, including joint angles, TCP position, error/warning code, etc. Refer to [RobotMsg.msg](./xarm_msgs/msg/RobotMsg.msg) for content details.  
&ensp;&ensp;Another option is subscribing to ***"/joint_states"*** topic, which is reporting in [JointState.msg](http://docs.ros.org/jade/api/sensor_msgs/html/msg/JointState.html), however, currently ***only "position" field is valid***; "velocity" is non-filtered numerical differentiation based on 2 adjacent position data, and "effort" feedback are current-based estimated values, not from direct torque sensor, so they are just for reference.
&ensp;&ensp;In consideration of performance, current update rate of above two topics are set at ***5Hz***.  

  'xarm7_server.launch'를 실행하여 실제 xArm 로봇과 연결한 사용자는 ***"xarm/xarm_states"*** 주제를 구독 하여 관절 각도, TCP 위치, 오류/경고 코드 등 현재 로봇 상태에 대한 피드백 정보를 얻을 수 있습니다. 내용에 대한 자세한 내용은 [RobotMsg.msg](https://github.com/HyeWon33/xarm_ros/blob/master/xarm_msgs/msg/RobotMsg.msg) 를 참조하십시오.
  또 다른 옵션은 ***"/joint_states"*** 주제를 구독하는 ***것인데*** , 이는 [JointState.msg](http://docs.ros.org/jade/api/sensor_msgs/html/msg/JointState.html) 에서 보고 되지만 현재 ***는 "위치" 필드만 유효합니다.***; "속도"는 2개의 인접 위치 데이터를 기반으로 한 필터링되지 않은 수치 미분이며 "노력" 피드백은 직접 토크 센서가 아닌 전류 기반 추정값이므로 참고용일 뿐입니다. 성능을 고려하여 위 2가지 항목의 현재 업데이트 속도는 ***5Hz*** 로 설정되어 있습니다.
  

#### Setting Tool Center Point Offset(only effective for xarm_api ROS service control): 도구 중심점 오프셋 설정(xarm_api ROS 서비스 제어에만 유효):
&ensp;&ensp;The tool tip point offset values can be set by calling service "/xarm/set_tcp_offset". Refer to the figure below, please note this offset coordinate is expressed with respect to ***default tool frame*** (Frame B), which is located at flange center, with roll, pitch, yaw rotations of (PI, 0, 0) from base frame (Frame A). 

 도구 팁 포인트 오프셋 값은 "/xarm/set_tcp_offset" 서비스를 호출하여 설정할 수 있습니다. 이하 도면을 참조 이것에 대하여 표현 좌표 오프셋 도와주세요 ***기본 툴 프레임*** 롤, 피치,베이스 프레임으로부터 (PI, 0, 0)의 요 회전 (과 플랜지 중심 위치 (프레임 B) 프레임 A).  예를 들어:
 
![xArmFrames](./doc/xArmFrames.png)  
&ensp;&ensp;For example:  
```bash
$ rosservice call /xarm/set_tcp_offset 0 0 20 0 0 0
```
&ensp;&ensp;This is to set tool frame position offset (x = 0 mm, y = 0 mm, z = 20 mm), and orientation (RPY) offset of ( 0, 0, 0 ) radians with respect to initial tool frame (Frame B in picture). ***Note this offset might be overwritten by xArm Stdudio if it is not consistent with the default value set in studio!*** It is recommended to do the same TCP default offset configuration in xArm studio if you want to use it alongside with ros service control.  

초기 공구 프레임(Frame B in 그림). ***이 오프셋은 스튜디오에 설정된 기본값과 일치하지 않는 경우 xArm Stdudio에서 덮어쓸 수 있습니다.*** ros 서비스 제어와 함께 사용하려면 xArm 스튜디오에서 동일한 TCP 기본 오프셋 구성을 수행하는 것이 좋습니다.

#### Clearing Errors:
&ensp;&ensp;Sometimes controller may report error or warnings that would affect execution of further commands. The reasons may be power loss, position/speed limit violation, planning errors, etc. It needs additional intervention to clear. User can check error code in the message of topic ***"xarm/xarm_states"*** . 


 때때로 컨트롤러는 추가 명령 실행에 영향을 줄 수 있는 오류 또는 경고를 보고할 수 있습니다. 그 이유는 정전, 위치/속도 제한 위반, 계획 오류 등일 수 있습니다. 이를 해결하려면 추가 개입이 필요합니다. 사용자는 ***"xarm/xarm_states"*** 주제의 메시지에서 오류 코드를 확인할 수 있습니다 .
  
```bash
$ rostopic echo /xarm/xarm_states
```
&ensp;&ensp;If it is non-zero, the corresponding reason can be found out in the user manual. After solving the problem, this error satus can be removed by calling service ***"/xarm/clear_err"*** with empty argument.

  0이 아닌 경우 해당 이유는 사용 설명서에서 찾을 수 있습니다. 문제를 해결한 후 빈 인수로 ***"/xarm/clear_err"*** 서비스를 호출하여 이 오류 상태를 제거할 수 있습니다 .
  
```bash
$ rosservice call /xarm/clear_err
```
&ensp;&ensp;If using Moveit!, call "**/xarm/moveit_clear_err**" instead to avoid the need of setting mode 1 again manually. 

Moveit!을 사용하는 경우 모드 1을 수동으로 다시 설정할 필요가 없도록 대신 " **/xarm/moveit_clear_err** "를 호출 하십시오.
  
```bash
$ rosservice call /xarm/moveit_clear_err
```

&ensp;&ensp;After calling this service, please ***check the err status again*** in 'xarm/xarm_states', if it becomes 0, the clearing is successful. Otherwise, it means the error/exception is not properly solved. If clearing error is successful, remember to ***set robot state to 0*** to make it ready to move again!   

  이 서비스를 호출한 후 'xarm/xarm_states'에서 ***err 상태를 다시 확인하여*** 0이 되면 클리어가 성공한 것이다. 그렇지 않으면 오류/예외가 제대로 해결되지 않았음을 의미합니다. 오류 해결에 성공하면 ***로봇 상태를 0*** 으로 설정하여 다시 이동할 수 있도록 하는 것을 잊지 마십시오!

#### Gripper Control:
&ensp;&ensp; If xArm Gripper (from UFACTORY) is attached to the tool end, the following services/actions can be called to operate or check the gripper.  

xArm Gripper(UFACTORY)가 공구 끝에 부착되어 있으면 다음 서비스/액션을 호출하여 그리퍼를 작동하거나 확인할 수 있습니다.

##### 1. Gripper services:  
(1) First enable the griper and configure the grasp speed:   먼저 그리퍼를 활성화하고 파악 속도를 구성합니다.
```bash
$ rosservice call /xarm/gripper_config 1500
```
&ensp;&ensp; Proper range of the speed is ***from 1 to 5000***. 1500 is used as an example. 'ret' value is 0 for success.  

속도의 적절한 범위는 ***1 ~ 5000*** 입니다. 1500이 예시로 사용됩니다. 'ret' 값은 성공의 경우 0입니다.

(2) Give position command (open distance) to xArm gripper:  그리퍼에 위치 명령(개방 거리)을 부여합니다.
```bash
$ rosservice call /xarm/gripper_move 500
```
&ensp;&ensp; Proper range of the open distance is ***from 0 to 850***. 0 is closed, 850 is fully open. 500 is used as an example. 'ret' value is 0 for success.  

열린 거리의 적절한 범위는 ***0 ~ 850*** 입니다. 0은 닫혀 있고 850은 완전히 열려 있습니다. 500이 예시로 사용됩니다. 'ret' 값은 성공을 위해 0입니다.
   

(3) To get the current status (position and error_code) of xArm gripper: 그리퍼의 현재 상태(위치 및 error_code)를 얻으려면:
```bash
$ rosservice call /xarm/gripper_state
```
&ensp;&ensp; If error code is non-zero, please refer to user manual for the cause of error, the "/xarm/clear_err" service can still be used to clear the error code of xArm Gripper.  

   오류 코드가 0이 아닌 경우 오류의 원인은 사용 설명서를 참조하십시오. "/xarm/clear_err" 서비스는 여전히 xArm Gripper의 오류 코드를 지우는 데 사용할 수 있습니다.

##### 2. Gripper action:
&ensp;&ensp; The xArm gripper move action is defined in [Move.action](/xarm_gripper/action/Move.action). The goal consists of target pulse position and the pulse speed. By setting "true" of "**use_gripper_action**" argument in xarm_bringup/launch/xarm7_server.launch, the action server will be started. Gripper action can be called by:  

   xArm 그리퍼 이동 동작은 [Move.action에](https://github.com/HyeWon33/xarm_ros/blob/master/xarm_gripper/action/Move.action) 정의되어 [있습니다](https://github.com/HyeWon33/xarm_ros/blob/master/xarm_gripper/action/Move.action) . 목표는 목표 펄스 위치와 펄스 속도로 구성됩니다. xarm_bringup/launch/xarm7_server.launch 의 " **use_gripper_action** " 인수를 "true"로 설정 하면 액션 서버가 시작됩니다. 그리퍼 작업은 다음과 같이 호출할 수 있습니다.
   
```bash
$ rostopic pub -1 /xarm/gripper_move/goal xarm_gripper/MoveActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  target_pulse: 500.0
  pulse_speed: 1500.0"

```
&ensp;&ensp; Alternatively: 또는:
```bash
$ rosrun xarm_gripper gripper_client 500 1500 
```

#### Vacuum Gripper Control:
&ensp;&ensp; If Vacuum Gripper (from UFACTORY) is attached to the tool end, the following service can be called to operate the vacuum gripper.  
진공 그리퍼(UFACTORY)가 공구 끝에 부착된 경우 다음 서비스를 호출하여 진공 그리퍼를 작동할 수 있습니다.

&ensp;&ensp;To turn on:  
```bash
$ rosservice call /xarm/vacuum_gripper_set 1
```
&ensp;&ensp;To turn off:  
```bash
$ rosservice call /xarm/vacuum_gripper_set 0
```
&ensp;&ensp;0 will be returned upon successful execution.   성공적으로 실행되면 0이 반환됩니다.


#### Tool Modbus communication: Tool Modbus communication: 도구 Modbus 통신:
If modbus communication with the tool device is needed, please first set the proper baud rate and timeout parameters through the "xarm/config_tool_modbus" service (refer to [ConfigToolModbus.srv](/xarm_msgs/srv/ConfigToolModbus.srv)). For example: 
```bash
$ rosservice call /xarm/config_tool_modbus 115200 20
```
The above command will configure the tool modbus baudrate to be 115200 bps and timeout threshold to be 20 **ms**. It is not necessary to configure again if these properties are not changed afterwards. **Please note** the first time to change the baud rate may return 1 (with error code 28), in fact it will succeed if the device is properly connected and there is no other exsisting controller errors. You can clear the error and call it once more to check if 0 is returned. Currently, only the following baud rates (bps) are supported: [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 1500000, 2000000, 2500000].  

Then the communication can be conducted like (refer to [SetToolModbus.srv](/xarm_msgs/srv/SetToolModbus.srv)):  
```bash
$ rosservice call /xarm/set_tool_modbus [0x01,0x06,0x00,0x0A,0x00,0x03] 7
```
First argument would be the uint8(unsigned char) data array to be sent to the modbus tool device, and second is the number of characters to be received as a response from the device. **This number should be the expected data byte length +1 (without CRC bytes)**. A byte with value of **0x09** would always be attached to the head if received from tool modbus, and the rest bytes are response data from the device. For example, with some testing device the above instruction would reply:  
```bash
ret: 0
respond_data: [9, 1, 6, 0, 10, 0, 3]
```
and actual feedback data frame is: [0x01, 0x06, 0x00, 0x0A, 0x00, 0x03], with the length of 6 bytes.   

위의 명령은 도구 modbus 전송 속도를 115200bps로 구성하고 시간 초과 임계값을 **20ms로 구성** 합니다. 이러한 속성이 나중에 변경되지 않으면 다시 구성할 필요가 없습니다. **유의하시기 바랍니다** 실제로 장치가 제대로 연결되어 있는지가 성공하고 다른 exsisting 컨트롤러 오류가 없다, 1 (오류 코드 28) 반환 할 수 있습니다 전송 속도를 변경하는 최초의 시간을. 오류를 지우고 한 번 더 호출하여 0이 반환되는지 확인할 수 있습니다. 현재 다음 전송 속도(bps)만 지원됩니다. [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 0,0,921600, 1000000, 1500000]

그런 다음 통신은 다음과 같이 수행될 수 있습니다( [SetToolModbus.srv](https://github.com/HyeWon33/xarm_ros/blob/master/xarm_msgs/srv/SetToolModbus.srv) 참조 ).

```
$ rosservice call /xarm/set_tool_modbus [0x01,0x06,0x00,0x0A,0x00,0x03] 7
```

First argument would be the uint8(unsigned char) data array to be sent to the modbus tool device, and second is the number of characters to be received as a response from the device. **This number should be the expected data byte length +1 (without CRC bytes)**. A byte with value of **0x09** would always be attached to the head if received from tool modbus, and the rest bytes are response data from the device. For example, with some testing device the above instruction would reply:



첫 번째 인수는 modbus 도구 장치로 보낼 uint8(unsigned char) 데이터 배열이고 두 번째 인수는 장치에서 응답으로 받을 문자 수입니다. **이 숫자는 예상 데이터 바이트 길이 +1(CRC 바이트 제외)이어야 합니다** . 값이 **0x09인** 바이트 는 도구 모드버스에서 수신하면 항상 헤드에 첨부되며 나머지 바이트는 장치의 응답 데이터입니다. 예를 들어, 일부 테스트 장치의 경우 위 명령은 다음과 같이 응답합니다.

```
ret: 0
respond_data: [9, 1, 6, 0, 10, 0, 3]
```

and actual feedback data frame is: [0x01, 0x06, 0x00, 0x0A, 0x00, 0x03], with the length of 6 bytes.

실제 피드백 데이터 프레임은 [0x01, 0x06, 0x00, 0x0A, 0x00, 0x03]이며 길이는 6바이트입니다.


# 6. Mode Change
&ensp;&ensp;xArm may operate under different modes depending on different controling methods. Current mode can be checked in the message of topic "xarm/xarm_states". And there are circumstances that demand user to switch between operation modes. 

  xArm은 제어 방법에 따라 다른 모드에서 작동할 수 있습니다. 현재 모드는 "xarm/xarm_states" 주제의 메시지에서 확인할 수 있습니다. 그리고 사용자가 작동 모드를 전환해야 하는 상황이 있습니다.

### 6.1 Mode Explanation

&ensp;&ensp; ***Mode 0*** : xArm controller (Position) mode.  
&ensp;&ensp; ***Mode 1*** : External trajectory planner (position) mode.  
&ensp;&ensp; ***Mode 2*** : Free-Drive (zero gravity) mode.  
&ensp;&ensp; ***Mode 3*** : Reserved.  
&ensp;&ensp; ***Mode 4*** : Joint velocity control mode.  
&ensp;&ensp; ***Mode 5*** : Cartesian velocity control mode.  

&ensp;&ensp; ***모드 0*** : xArm 컨트롤러(위치) 모드.
&ensp;&ensp; ***모드 1*** : 외부 궤적 플래너(위치) 모드.
&ensp;&ensp; ***모드 2*** : 프리 드라이브(무중력) 모드.
&ensp;&ensp; ***모드 3*** : 예약됨.  
&ensp;&ensp; ***모드 4*** : 관절 속도 제어 모드.
&ensp;&ensp; ***모드 5*** : 직교 속도 제어 모드.

&ensp;&ensp;***Mode 0*** is the default when system initiates, and when error occurs(collision, overload, overspeed, etc), system will automatically switch to Mode 0. Also, all the motion plan services in [xarm_api](./xarm_api/) package or the [SDK](https://github.com/xArm-Developer/xArm-Python-SDK) motion functions demand xArm to operate in Mode 0. ***Mode 1*** is for external trajectory planner like Moveit! to bypass the integrated xArm planner to control the robot. ***Mode 2*** is to enable free-drive operation, robot will enter Gravity compensated mode, however, be sure the mounting direction and payload are properly configured before setting to mode 2. ***Mode 4*** is to control arm velocity in joint space. ***Mode 5*** is to control arm (linear) velocity in Cartesian space.

  ***모드 0*** 은 시스템 시작 시 기본값이며 오류(충돌, 과부하, 과속 등) 발생 시 시스템은 자동으로 모드 0으로 전환됩니다. 또한[ xarm_api](https://github.com/HyeWon33/xarm_ros/blob/master/xarm_api) 패키지 또는[ SDK](https://github.com/xArm-Developer/xArm-Python-SDK) 모션 기능의모든 모션 계획 서비스는xArm이 작동하도록 요구합니다. 모드 0에서. ***모드 1*** 은 Moveit과 같은 외부 궤적 플래너를 위한 것입니다! 통합 xArm 플래너를 우회하여 로봇을 제어합니다. ***모드 2*** 는 자유 구동 작동을 가능하게 하고 로봇은 중력 보상 모드로 들어가지만 모드 2로 설정하기 전에 장착 방향과 페이로드가 올바르게 구성되어 있는지 확인하십시오. ***모드 4*** 는 관절 공간에서 팔 속도를 제어하는 것입니다. ***모드 5*** 데카르트 공간에서 팔(선형) 속도를 제어하는 것입니다.

### 6.2 Proper way to change modes:  
&ensp;&ensp;If collision or other error happens during the execution of a Moveit! planned trajectory, Mode will automatically switch from 1 to default mode 0 for safety purpose, and robot state will change to 4 (error state). The robot will not be able to execute any Moveit command again unless the mode is set back to 1. The following are the steps to switch back and enable Moveit control again:  

  Moveit! 실행 중 충돌 또는 기타 오류가 발생한 경우! 계획된 궤적, 모드는 안전을 위해 1에서 기본 모드 0으로 자동 전환되고 로봇 상태는 4(오류 상태)로 변경됩니다. 모드가 다시 1로 설정되지 않으면 로봇은 Moveit 명령을 다시 실행할 수 없습니다. 다음은 다시 전환하고 Moveit 제어를 활성화하는 단계입니다.

&ensp;&ensp;(1) Make sure the objects causing the collision are removed.   충돌을 일으키는 물체가 제거되었는지 확인하십시오.
&ensp;&ensp;(2) clear the error:   오류 지우기:
```bash
$ rosservice call /xarm/clear_err
```
&ensp;&ensp;(3) switch to the desired mode (Mode 2 for example), and set state to 0 for ready: 원하는 모드(예: 모드 2)로 전환하고 준비 상태를 0으로 설정합니다.
```bash
$ rosservice call /xarm/set_mode 2

$ rosservice call /xarm/set_state 0
```

# 7. Other Examples
&ensp;&ensp;There are some other application demo examples in the [example package](./examples), which will be updated in the future, feel free to explore it.

[예제 패키지](https://github.com/HyeWon33/xarm_ros/blob/master/examples)  에 몇 가지 다른 애플리케이션 데모 예제 가 있습니다. 이 [패키지](https://github.com/HyeWon33/xarm_ros/blob/master/examples) 는 향후 업데이트될 예정이므로 자유롭게 탐색하십시오.
