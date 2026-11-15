# 第四关：点云检测与飞行

无人机从起点起飞，识别前方四根柱子，在它们之间选择通道，穿过后到终点降落。柱子的位置和半径会随机变化，侧墙贴近最外侧柱子，不能从两边绕行。

## 4.1 我们提供什么

本工作空间提供仿真场景、随机柱子、飞行主控和检测结果接口。主控负责起飞、选择通道、移动和降落。

学生镜像中的点云输入沿用第三关的 `/cloud_registered`。主控统一向回调提供 `sensor_msgs/PointCloud2`，坐标为世界系 ENU，与 MAVROS 本地坐标对齐，单位为米。

## 4.2 你需要提供什么

自行编写点云回调，实现下面的接口。声明见 [callback.hpp](../src/tutorial_pcl_avoid/include/pcl_avoid/callback.hpp)：

```cpp
namespace pcl_avoid {
void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);
}
```

`msg` 是本帧点云消息。主控收到点云后会调用该回调，你需要根据输入估计柱子的位置和半径，通过下面的接口提交本帧结果：

```cpp
void submitDetection(const ObstacleList& obstacles);
```

结果类型定义在 [obstacle.hpp](../src/tutorial_pcl_avoid/include/pcl_avoid/obstacle.hpp)：

```cpp
struct Obstacle {
  float x;       // 近似中心的机体系 x 坐标，米
  float y;       // 近似中心的机体系 y 坐标，米
  float radius;  // 估计半径，米
};
using ObstacleList = std::vector<Obstacle>;
```

位置和半径允许近似估计，不要求精确拟合几何圆心。以正确识别柱子、判断通道并完成飞行为目标。

输出采用机体系坐标，原点为机体原点，x 朝前、y 朝左、z 朝上。坐标与半径须为有限值，半径大于零。列表顺序不限，没有可用结果时提交空列表，不要沿用上一帧结果。检测结果必须来自点云，不得读取测试真值或写死柱子的位置、半径。

另提供一个可选的坐标转换工具，将输入消息转换为机体系点云，主控在悬停检测期间启用这个工具，成功时返回 `true`：

```cpp
bool cloudToBody(const sensor_msgs::PointCloud2& msg, Cloud& output);
```

在 `src/tutorial_pcl_avoid/student/` 下自行创建 `.cpp` 文件，编译时会自动收集该目录及其子目录中的源码。算法、函数划分和数据结构由你决定，只需满足上述回调和输出约定。

## 4.3 编译与运行

实现回调后，在镜像环境中进入本工作空间根目录，编译并启动：

```zsh
catkin_make
bash src/tutorial_pcl_avoid/shell/start.sh
```

脚本参照 `raicom26_ws` 的方式，通过 tmux 一起启动 ROS 核心、仿真、主控和监控。启动后焦点在主控面板；场景与飞控就绪后，无人机会自动进入 OFFBOARD、解锁并起飞。

`Core_Sim` 窗口显示核心与仿真，`Control` 窗口显示主控、位置、飞控状态和点云频率。使用 `Ctrl+b` 后按 `n` 切换窗口。

复现同一组柱子时，在启动命令后加 `seed:=7`。脚本优先使用学生镜像中的 `simulation`，本地开发环境则自动选择 `tutorial_gazebo`。

结束任务时，在另一个终端执行：

```zsh
bash src/tutorial_pcl_avoid/shell/start.sh --stop
```

修改 C++ 后重新编译，并结束旧任务后再启动。再次执行启动脚本会返回现有会话，不会重启其中的程序。

离线检查不启动仿真：

```zsh
bash src/tutorial_pcl_avoid/shell/start.sh test
```
