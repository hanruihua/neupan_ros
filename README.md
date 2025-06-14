# neupan_ros

This is the ROS wrapper for [NeuPAN Planner](https://github.com/hanruihua/neupan).

## Prerequisites
- Ubuntu 20.04
- ROS Noetic
- Python >= 3.10
- Installed [NeuPAN Planner](https://github.com/hanruihua/neupan).

> [!Note] 
> If you want to use python3.8 to run this project without virtual environment, you can install the py38 branch of NeuPAN Planner: https://github.com/hanruihua/NeuPAN/tree/py38

## Installation

```bash
mkdir -p ~/neupan_ws/src
cd ~/neupan_ws/src
git clone https://github.com/hanruihua/neupan_ros
cd ~/neupan_ws && catkin_make
cd ~/neupan_ws/src/neupan_ros 
sh source_setup.sh && source ~/neupan_ws/devel/setup.sh && rosdep install neupan_ros 
```

## Demonstration

### Dynamic collision avoidance

We provide the dynamic collision avoidance examples in Gazebo shown as follows. To run these examples, please see [example/gazebo_limo](https://github.com/hanruihua/neupan_ros/tree/main/example/gazebo_limo) for detail.

https://github.com/user-attachments/assets/1d5eb028-0d22-4741-8899-40a3ea7caab4

## Node API 

Publish Topics

| Topic Name             | Message Type                     | Description                                |
| ---------------------- | -------------------------------- | ------------------------------------------ |
| `/neupan_cmd_vel`      | `geometry_msgs/Twist`            | The velocity command to the robot.         |
| `/neupan_plan`         | `nav_msgs/Path`                  | The NeuPAN planned path.                   |
| `/neupan_initial_path` | `nav_msgs/Path`                  | The initial path for NeuPAN visualization. |
| `/neupan_ref_state`    | `nav_msgs/Path`                  | The current reference state visualization. |
| `/dune_point_markers`  | `visualization_msgs/MarkerArray` | The DUNE points markers visualization.     |
| `/nrmp_point_markers`  | `visualization_msgs/MarkerArray` | The NRMP points markers visualization.     |
| `/robot_marker`        | `visualization_msgs/Marker`      | The robot marker visualization.            |

Subscribe Topics

| Topic Name      | Message Type                | Description                                                                                        |
| --------------- | --------------------------- | -------------------------------------------------------------------------------------------------- |
| `/scan`         | `sensor_msgs/LaserScan`     | The laser scan data.                                                                               |
| `/initial_path` | `nav_msgs/Path`             | Set the initial path directly for NeuPAN.                                                                   |
| `/neupan_goal`  | `geometry_msgs/PoseStamped` | Set the goal for NeuPAN, the initial path will be generated from the current robot state to this goal point. |
| `/neupan_waypoints`    | `nav_msgs/Path`             | Set the waypoints for NeuPAN, the initial path will be generated from the current robot state to these waypoints. |

> [!NOTE]
> To set (refresh) the initial path, you can use one of the above three topics: "initial_path", "neupan_goal" or "neupan_waypoints". Please set `refresh_initial_path` to `True` if you want to update the initial path continuously.

ROS Parameters

| Parameter Name          | Type / Default Value | Description                                              |
| ----------------------- | -------------------- | -------------------------------------------------------- |
| `~config_file`          | `str` / None         | The path of the NeuPAN config YAML file.                 |
| `~map_frame`            | `str` / `map`        | The map frame name.                                      |
| `~base_frame`           | `str` / `base_link`  | The base frame name.                                     |
| `~lidar_frame`          | `str` / `laser_link` | The lidar frame name.                                    |
| `~marker_size`          | `float` / `0.05`     | The marker size to show the DUNE points and NRMP points. |
| `~marker_z`             | `float` / `1.0`      | The marker z to show the height of the robot.            |
| `~scan_angle_range`     | `str` / `-3.14 3.14` | The angle range of the laser scan.                       |
| `~scan_downsample`      | `int` / `1`          | The downsample rate of the laser scan.                   |
| `~scan_range`           | `str` / `0.0, 5.0`   | The distance range of the laser scan.                    |
| `~dune_checkpoint`      | `str` / `None`       | The path of the DUNE checkpoint file.                    |
| `~refresh_initial_path` | `bool` / `False`     | Whether to refresh the initial path so that the planner can update the initial path from goal, waypoints or given path.                     |
| `~flip_angle`           | `bool` / `False`     | Whether to flip the angle of the scan data.              |
| `~include_initial_path_direction` | `bool` / `False`     | Whether to set the initial path direction from the given path. If `False`, the points gradient will be used as the initial path direction.            |

## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).

## Citation

If you find this code or paper is helpful, please kindly star :star: this repository and cite our paper by the following BibTeX entry:

```bibtex
@ARTICLE{10938329,
  author={Han, Ruihua and Wang, Shuai and Wang, Shuaijun and Zhang, Zeqing and Chen, Jianjun and Lin, Shijie and Li, Chengyang and Xu, Chengzhong and Eldar, Yonina C. and Hao, Qi and Pan, Jia},
  journal={IEEE Transactions on Robotics}, 
  title={NeuPAN: Direct Point Robot Navigation With End-to-End Model-Based Learning}, 
  year={2025},
  volume={41},
  number={},
  pages={2804-2824},
  doi={10.1109/TRO.2025.3554252}}
```

