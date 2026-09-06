#!/bin/bash
# publish_phase1.sh — 第一阶段发布脚本
# 前置: gh auth login 已完成，且账号对 Romindly-Dev 组织有建仓库权限
#
# 动作:
#   1. 在 Romindly-Dev 下创建 3 个自建仓库并推送本地代码
#   2. fork 第一阶段的 4 个上游仓库到 Romindly-Dev（不克隆，仅 fork）
set -euo pipefail

ORG=Romindly-Dev
SRC=/home/iot/workspace/ws_romindly/src
GH=${GH:-gh}

echo "== 检查登录状态 =="
$GH auth status

# ---------- 自建仓库 ----------
declare -A OWN_REPOS=(
  [romindly_ros1_workspace]="ROS1 定位·导航·基础学习与部署套件总入口: 全套中文教程 + 一键拉取 + Docker"
  [romindly_ros1_tutorials]="ROS1 (Noetic) 基础示例代码: 话题/服务/Action/TF2/URDF/launch 七个包"
  [romindly_robot_bringup]="x86 边缘计算单元机器人启动包: 传感器集成 launch 与开机自启"
)

for repo in "${!OWN_REPOS[@]}"; do
  echo "== 创建并推送 $ORG/$repo =="
  if ! $GH repo view "$ORG/$repo" &>/dev/null; then
    $GH repo create "$ORG/$repo" --public --description "${OWN_REPOS[$repo]}"
  fi
  cd "$SRC/$repo"
  git remote get-url origin &>/dev/null || git remote add origin "https://github.com/$ORG/$repo.git"
  git push -u origin main
done

# ---------- fork 上游 (第一阶段: 基础与仿真) ----------
UPSTREAMS=(
  ros/ros_tutorials
  ROBOTIS-GIT/turtlebot3
  ROBOTIS-GIT/turtlebot3_msgs
  ROBOTIS-GIT/turtlebot3_simulations
)

for up in "${UPSTREAMS[@]}"; do
  name=${up#*/}
  echo "== fork $up -> $ORG/$name =="
  if ! $GH repo view "$ORG/$name" &>/dev/null; then
    $GH repo fork "$up" --org "$ORG" --clone=false
  else
    echo "   已存在，跳过"
  fi
done

echo "== 完成。仓库列表: =="
$GH repo list "$ORG"
