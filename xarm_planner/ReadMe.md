# Package Introduction
&ensp;&ensp;This package is intended to provide users a demo programming interface to use moveit!, instead of just using the GUI. To use the API better, users are encouraged to go through [Moveit tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/).  
&ensp;&ensp;Inside the package, 'xarm_simple_planner' is just a basic implementation of the Move_group interface, if higher level configurations (constraints, switch kinematic solver or planners, etc) are needed, user can fully explore Moveit abilities and implement a sofisticated version.  

 이 패키지는 사용자에게 GUI를 사용하는 대신 moveit!를 사용할 수 있는 데모 프로그래밍 인터페이스를 제공하기 위한 것이다. API를 더 잘 사용하기 위해서는 Moveit 튜토리얼을 이용하도록 권장된다.
패키지 내부에서 'xarm_simple_planner'는 Move_group 인터페이스의 기본 구현에 불과하며, 보다 높은 수준의 구성(불완전, 스위치 키네마틱솔버 또는 플래너 등)이 필요할 경우 사용자는 Moveit 능력을 충분히 탐색하고 소박한 버전을 구현할 수 있다.


# Usage 사용법
## Launch the simple planner node:
If you want to try it in simulation, run:
```bash
   $ roslaunch xarm_planner xarm_planner_rviz_sim.launch robot_dof:=<7|6|5> add_gripper:=<true|false> add_vacuum_gripper:=<true|false>
```
Or, if you would work with real xArm, run:  
```bash
   $ roslaunch xarm_planner xarm_planner_realHW.launch robot_ip:=<your controller box LAN IP address> robot_dof:=<7|6|5> add_gripper:=<true|false> add_vacuum_gripper:=<true|false>
```
Argument 'robot_dof' specifies the number of joints of your xArm (default is 7). 'add_gripper' and 'add_vacuum_gripper' are for the cases with UF end-effector attached, only one end-effector can be attached. 

'robot_dof' 인수는 xArm의 조인트 수를 지정한다(기본값은 7). 'add_griper'와 'add_vacuum_griper'는 UF 엔드 이펙터가 부착된 경우로, 엔드 이펙터를 하나만 부착할 수 있다.


This node can provide services for planning request in Cartesian target and joint space target. Service definition can be found in srv folder. User can call the services to let planner solve the path to specified target point, and retrieve the boolean result as successful or not. Once the node is launched, user can try in command-line first, something like:  

이 노드는 데카르트 목표와 공동 공간 목표에서 계획 요청을 위한 서비스를 제공할 수 있다. 서비스 정의는 srv 폴더에서 찾을 수 있다. 사용자는 서비스를 호출하여 플래너가 지정된 목표 지점에 대한 경로를 해결하도록 하고, 성공 여부에 따라 부울 결과를 검색할 수 있다. 일단 노드가 실행되면 사용자는 명령줄을 먼저 사용할 수 있다.

## For joint-space planning request:  공동 공간 계획 요청의 경우:
```bash
   $ rosservice call xarm_joint_plan 'target: [1.0, -0.5, 0.0, -0.3, 0.0, 0.0, 0.5]'
```
The target elements in this case correspond to each joint target angle in radians, number of elements is same with the DOF.  

이 경우 대상 요소는 라디안 단위의 각 접합 대상 각도에 해당하며, 요소의 수는 DOF와 동일하다.

## For Cartesian-space point-to-point planning request:  데카르트-공간 점 대 점 계획 요청의 경우:
```bash
   $ rosservice call xarm_pose_plan 'target: [[0.28, 0.2, 0.2], [1.0, 0.0, 0.0, 0.0]]'
```
The separated fields for Cartesian target correspond to tool frame position (x, y, z) in ***meters*** and orientation ***Quaternions*** (x, y, z, w).  
Note that this motion is still point-to-point, meaning the trajectory is NOT a straight line.  

데카르트 표적에 대해 분리된 필드는 미터 단위의 도구 프레임 위치(x, y, z) 및 방향 쿼터니언(x, y, z, w)에 해당한다.
이 동작은 여전히 점 대 점으로, 궤적이 직선이 아님을 의미한다.


## For Cartesian straight line planning request: 데카르트 직선 계획 요청의 경우:
```bash
   $ rosservice call xarm_straight_plan 'target: [[0.28, 0.2, 0.2], [1.0, 0.0, 0.0, 0.0]]'
```
Command data units are the same with above Cartesian pose command. If planned succesfully, end-effector trajectory will be a straight-line in space. Note that the velocity change may not be as expected during execution. Please refer to MoveGroupInterface documentation to make your modifications to the code if necessary.  

명령 데이터 단위는 위의 카르테시안 포즈 명령과 동일하다. 성공적으로 계획되면 엔드 이펙터 궤적은 우주에서 직선 궤적이 될 것이다. 실행 중에 속도 변화가 예상과 다를 수 있다는 점에 유의하십시오. 그룹 이동을 참조하십시오.필요한 경우 코드를 수정하기 위한 인터페이스 문서.


After calling the above planning services, a boolean result named 'success' will be returned.  

위의 계획 서비스를 호출한 후 '성공'이라는 부울 결과가 반환된다.


### Quaternion calculation tips: 쿼터니온 계산 팁:
If you are not familiar with conversion from (roll, pitch, yaw) to quaternions, refer to [this page](http://wiki.ros.org/tf2/Tutorials/Quaternions#Think_in_RPY_then_convert_to_quaternion).

(롤, 피치, 요)에서 쿼터니온으로의 변환에 익숙하지 않은 경우 이 페이지를 참조하십시오.

## For Execution of planned trajectory:  계획된 궤적 실행의 경우:

***Notice: Use Cartesian planning with special care, since trajectory from Moveit (OMPL) planner can be highly random and not necessarily the optimal (closest) solution, Do check it in Rviz befroe confirm to move!*** 

참고: Moveit(OMPL) Planner의 궤적은 매우 랜덤할 수 있고 반드시 최적의(최적) 솔루션이 아니므로 Rviz Berfroe에서 확인하여 이동하십시오!


If solution exists and user want to execute it on the robot, it can be done by  service call (**recommended**) or topic message. 

솔루션이 존재하고 사용자가 로봇에서 이를 실행하고자 하는 경우 서비스 호출(권장) 또는 주제 메시지로 수행할 수 있다.

### Execute by service call (Blocking) 서비스 호출로 실행(차단)
Call the 'xarm_exec_plan' service with a request data of 'true', the latest planned path will be executed, the service will return after finishing the execution:  

'true'의 요청 데이터를 사용하여 'xarm_exec_plan' 서비스에 전화하십시오. 계획된 최신 경로가 실행되며, 실행이 완료된 후 서비스가 반환됨:

```bash
   $ rosservice call xarm_exec_plan 'true'
```

### Execute by topic (Non-blocking) 항목별 실행(비차단)
Just publish a message (type: std_msgs/Bool) to the topic "/xarm_planner_exec", the boolean data should be 'true' to launch the execution, it returns immediately and does not wait for finish:  

메시지(유형: std_msgs/Bool)를 "/xarm_planner_exec" 항목에 게시하면, 실행이 시작되려면 부울 데이터가 'true'여야 하며, 즉시 반환되며 완료되기를 기다리지 않는다.

```bash
   $ rostopic pub -1 /xarm_planner_exec std_msgs/Bool 'true'
```

Alternative way of calling services or publish messages, is to do it programatically. User can refer to ROS [tutorial1](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29) and [tutorial2](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) to find out how to do it, or refer to the 'xarm_simple_planner_test.cpp' in the src folder as an example.  

서비스를 호출하거나 메시지를 게시하는 다른 방법은 프로그래밍 방식으로 수행하는 것이다. 사용자는 ROS 자습서1과 자습서2를 참조하여 어떻게 하는지 알아보거나, src 폴더의 'xarm_simple_planner_test.cpp'를 예로 들 수 있다.

To run the test program ( ***for xArm7 only***, user can modify the command list for other models), after launching the simple planner:

단순 플래너를 실행한 후 테스트 프로그램(xArm7의 경우, 사용자는 다른 모델의 명령 목록을 수정할 수 있음)을 실행하려면:

```bash
   $ rosrun xarm_planner xarm_simple_planner_test
```
The program will execute three hard-coded joint space target, ***MAKE SURE THERE ARE PLENTY SURROUNDING SPACES BEFORE EXECUTING THIS!***


이 프로그램을 실행하기 전에 하드 코딩된 세 개의 공동 공간 목표를 실행한다.

### Visualization of planned trajectory 계획된 궤적 시각화
Refer to issue [#57](https://github.com/xArm-Developer/xarm_ros/issues/57), now xArm and end-effector descriptions are re-configured to enable the visualization of TCP trajectory upon successful planning. As is shown below:   

#57 문제를 참조하십시오. 이제 xArm 및 엔드 이펙터 설명은 성공적인 계획 수립 시 TCP 궤적을 시각화할 수 있도록 다시 구성된다. 아래 그림과 같이:

![VISUAL_TRAJ1](../doc/visual_traj1.png)
![VISUAL_TRAJ2](../doc/visual_traj2.png)
