# utils

本目录用于存放项目依赖的外部工具库，使仓库自包含（无需依赖其他工作空间的残留构建产物）。

| 库 | 来源 | 说明 |
|---|---|---|
| `quadrotor_msgs` | [hku-mars/SUPER](https://github.com/hku-mars/SUPER) 的 `mars_uav_sim/mars_quadrotor_msgs`（仅保留 ROS1 部分） | `PositionCommand`、`PolynomialTrajectory` 等消息定义，`super_planner` / `planner_ctrl` 依赖 |

后续计划迁入的公共代码（消除各包重复拷贝）：

- `color_msg_utils.hpp`（当前在 `super_planner` / `rog_map` 中各有一份拷贝）
- `fmt`（当前在 `super_planner` / `rog_map` 中各有一份拷贝）

## 注意

- 包名与目录解耦：`utils/quadrotor_msgs` 的 catkin 包名就是 `quadrotor_msgs`，
  话题 `/planner_cmd` 上的消息类型保持 `quadrotor_msgs/PositionCommand` 不变，
  与 `planner_ctrl` 及其他规划器（ego_planner、fast_planner 等）保持互通。
