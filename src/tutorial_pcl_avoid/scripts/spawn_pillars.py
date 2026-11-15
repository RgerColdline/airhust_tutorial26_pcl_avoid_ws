#!/usr/bin/env python3
"""生成任务场景；全部模型生成成功后发布就绪标志。"""

import random

import numpy as np
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

PILLAR_X = 4.0
PILLAR_COUNT = 4
PASSABLE_GAP = (1.9, 2.3)   # 中心距
BLOCKED_GAP = (0.6, 0.9)    # 中心距
PILLAR_RADIUS = (0.15, 0.25)


def make_layout(rng):
    """和 data/make_pillar_frame.py 用同一套随机数，seed 相同布局就相同。"""
    n = PILLAR_COUNT
    pass_idx = int(rng.integers(0, n - 1))
    gaps = []
    for i in range(n - 1):
        if i == pass_idx:
            gaps.append(rng.uniform(*PASSABLE_GAP))
        else:
            gaps.append(rng.uniform(*BLOCKED_GAP))

    ys = [0.0]
    for gap in gaps:
        ys.append(ys[-1] + gap)
    shift = rng.uniform(-0.4, 0.4) - (ys[0] + ys[-1]) / 2.0
    ys = [y + shift for y in ys]

    pillars = []
    for i in range(n):
        pillars.append(
            (
                PILLAR_X + rng.uniform(-0.1, 0.1),
                ys[i],
                rng.uniform(*PILLAR_RADIUS),
            )
        )
    return pillars


def pillar_sdf(name, radius, length=2.0):
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="%s">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><cylinder><radius>%.4f</radius><length>%.4f</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>%.4f</radius><length>%.4f</length></cylinder></geometry>
        <material><script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Grey</name>
        </script></material>
      </visual>
    </link>
  </model>
</sdf>
""" % (name, radius, length, radius, length)


# 两侧只保留 20 cm 边缝，不能从最外侧柱子旁边绕过。
SIDE_CLEARANCE = 0.20
WALL_THICKNESS = 0.15
WALL_HEIGHT = 2.5


def make_walls(pillars):
    lower = min(y - radius for _, y, radius in pillars) - SIDE_CLEARANCE
    upper = max(y + radius for _, y, radius in pillars) + SIDE_CLEARANCE
    middle = (lower + upper) / 2.0
    width = upper - lower + 2.0 * WALL_THICKNESS
    return [
        ("boundary_south", (4.0, lower - WALL_THICKNESS / 2.0), (12.15, WALL_THICKNESS)),
        ("boundary_north", (4.0, upper + WALL_THICKNESS / 2.0), (12.15, WALL_THICKNESS)),
        ("boundary_west", (-2.0, middle), (WALL_THICKNESS, width)),
        ("boundary_east", (10.0, middle), (WALL_THICKNESS, width)),
    ]


def wall_sdf(name, size):
    sx, sy = size
    return f'''<sdf version="1.6"><model name="{name}"><static>true</static>
<link name="wall"><collision name="collision"><geometry><box>
<size>{sx} {sy} {WALL_HEIGHT}</size></box></geometry></collision>
<visual name="visual"><geometry><box><size>{sx} {sy} {WALL_HEIGHT}</size></box></geometry>
<material><script><uri>file://media/materials/scripts/gazebo.material</uri>
<name>Gazebo/Grey</name></script></material></visual></link></model></sdf>'''


def main():
    rospy.init_node("spawn_pillars")

    seed = rospy.get_param("~seed", 0)
    if seed == 0:
        seed = random.randrange(1, 1000000)
    rng = np.random.default_rng(seed)

    pillars = make_layout(rng)
    rospy.loginfo("布局 seed=%d", seed)

    ready = rospy.Publisher("/pcl_avoid/scene_ready", Bool, queue_size=1, latch=True)
    ready.publish(False)
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    for i, (x, y, r) in enumerate(pillars):
        name = "pillar_%d" % i
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 1.0  # 高 2m，底面位于地面 z=0。
        pose.orientation.w = 1.0
        try:
            response = spawn(name, pillar_sdf(name, r), "", pose, "world")
            if not response.success:
                rospy.logerr("生成柱子失败: %s", response.status_message)
                return
        except rospy.ServiceException as e:
            rospy.logerr("放柱子失败: %s", e)
            return

    for name, (x, y), size in make_walls(pillars):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = WALL_HEIGHT / 2.0
        pose.orientation.w = 1.0
        try:
            response = spawn(name, wall_sdf(name, size), "", pose, "world")
            if not response.success:
                rospy.logerr("生成围墙失败: %s", response.status_message)
                return
        except rospy.ServiceException as e:
            rospy.logerr("生成围墙失败: %s", e)
            return

    ready.publish(True)
    rospy.loginfo("任务场景已就绪。")

    rospy.spin()


if __name__ == "__main__":
    main()
