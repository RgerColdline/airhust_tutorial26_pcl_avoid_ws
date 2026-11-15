# AirHUST 2026 梯队任务：点云方向第四关

根据仿真点云识别柱子，让无人机从通道中穿过并降落。

输入数据、回调接口、输出约定和启动方式见 [任务说明](docs/stage4-avoid.md)。

```text
airhust_tutorial26_pcl_avoid_ws/
├── docs/                         # 任务说明
├── data/                         # 离线检查数据
└── src/tutorial_pcl_avoid/
    ├── student/                  # 在这里自行创建实现源码
    ├── include/pcl_avoid/         # 接口与主控声明
    ├── src/                      # 飞行主控与结果检查程序
    ├── config/                   # 飞行配置
    ├── shell/start.sh            # 主控、仿真与离线检查
    ├── launch/                   # 启动文件
    ├── worlds/                   # 场景
    └── scripts/                  # 场景生成脚本
```
