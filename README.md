# pka2026_sentry_nav（包含决策部分串口部分）

这是福建师范大学pikachu战队2026赛季导航，参考了北极熊战队的导航算法。

## 一、如何使用

#### 1.1 Setup Environment

- Ubuntu 22.04

- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

- 配套仿真包（Option）：[rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator)

- Install [small_icp](https://github.com/koide3/small_gicp):

  ```bash
  sudo apt install -y libeigen3-dev libomp-dev
  
  git clone https://github.com/koide3/small_gicp.git
  cd small_gicp
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
  sudo make install
  ```

### 第三方依赖库

livox-SDK2:https://github.com/Livox-SDK/Livox-SDK2.git

livox_ros_driver2: https://github.com/Livox-SDK/livox_ros_driver2.git

small_gicp: https://github.com/koide3/small_gicp.git

ndt_omp: https://github.com/koide3/ndt_omp.git

### 配置安装

安装依赖项

```
sudo apt install cmake
sudo apt install ros-humble-perception-pcl \
         ros-humble-pcl-msgs \
         ros-humble-vision-opencv \
         ros-humble-xacro
sudo apt install libpcap-dev
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev
sudo apt-get install libgeographic-dev
sudo apt install ros-humble-nav2-*
```



#### 1.2 Create Workspace

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git clone --recursive https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav.git src/pb2025_sentry_nav
```

下载先验点云:

先验点云用于 point_lio 和 small_gicp，由于点云文件体积较大，故不存储在 git 中，请前往 [FlowUs](https://flowus.cn/lihanchen/share/87f81771-fc0c-4e09-a768-db01f4c136f4?code=4PP1RS) 下载。

> 当前 point_lio with prior_pcd 在大场景的效果并不好，比不带先验点云更容易飘，待 Debug 优化

#### 1.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> [!NOTE]
> 推荐使用 --symlink-install 选项来构建你的工作空间，因为 pb2025_sentry_nav 广泛使用了 launch.py 文件和 YAML 文件。这个构建参数会为那些非编译的源文件使用符号链接，这意味着当你调整参数文件时，不需要反复重建，只需要重新启动即可。

#### 1.4 Running

可使用以下命令启动，在 RViz 中使用 `Nav2 Goal` 插件发布目标点。

#### 2.3.1 仿真

单机器人：

导航模式：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
world:=rmuc_2025 \
slam:=False
```

建图模式：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
slam:=True
```

保存栅格地图：`ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>  --ros-args -r __ns:=/red_standard_robot1`

多机器人 (实验性功能) :

当前指定的初始位姿实际上是无效的。TODO: 加入 `map` -> `odom` 的变换和初始化

```bash
ros2 launch pb2025_nav_bringup rm_multi_navigation_simulation_launch.py \
world:=rmul_2024 \
robots:=" \
red_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
blue_standard_robot1={x: 5.6, y: 1.4, yaw: 3.14}; \
"
```

#### 2.3.2 实车

建图模式：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=True
```

保存栅格地图：`ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>  --ros-args -r __ns:=/red_standard_robot1`

导航模式：

注意修改 `world` 参数为实际地图的名称

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=<YOUR_WORLD_NAME> \
slam:=False \
use_robot_state_pub:=True
```

## 二、日志（继承25赛季）

(已更新到哨兵pc上！！！)
20250313：已更新适配联盟赛串口

20250313：由于 NAV2 humble 发行版出于避免破坏原有接口的原因，依然使用 Twist 类型（不含时间戳），humble 往后的版本才使用 TwistStamped，导致无法直接实现 cmd_vel 与 odometry 的时间戳对齐。因此，本功能包暂时订阅 local_plan 话题（由局部路径规划器发布），以获取时间戳，将它的时间戳视为 cmd_vel 的时间戳，以间接实现时间戳对齐。所以把北极熊最新的fake_transform移植了进来。

20250314：添加心跳节点!

20251228：修改了cmakelist.txt

20260112：编译时必须加release，不然无法发挥全部性能；修改重定位函数，新增任意点启动功能，但相关文件生成部分还没写出来，故设置一个开关用来开关任意点启动。



