# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    node = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        namespace="",
        output="screen",
        remappings=remappings,
        parameters=[
            {
                # 新增的模式控制与调试开关
                "use_3d_method": True,    # 开启时使用 NDT+SmallGICP 3D重定位，关闭时使用 OpenCV 特征匹配2D重定位
                "debug": True,            # 统一的调试开关 (输出时间、误差等信息，如果是OpenCV还会保存图片在 /tmp)
                "map_topic": "map",       # nav2 发布的 2D 栅格地图话题

                # 基础参数
                "num_threads": 4,
                "num_neighbors": 10,
                "global_leaf_size": 0.25,
                "registered_leaf_size": 0.25,
                "max_dist_sq": 1.0,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "",
                "lidar_frame": "",
                "prior_pcd_file": "",
                "input_cloud_topic": "cloud_registered",
            }
        ],
    )

    return LaunchDescription([node])