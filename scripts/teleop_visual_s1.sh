#!/bin/bash

# RoboMaster S1 Visual Teleoperation with RViz
# RVizでロボットを可視化しながら遠隔操作するスクリプト

echo "=== RoboMaster S1 Visual Teleoperation with RViz ==="
echo ""

# ROS2環境のセットアップ
source /home/ikuo/s1_ws/install/setup.bash

echo "🤖 S1ドライバーを起動中..."
ros2 run s1_driver s1_driver_node &
DRIVER_PID=$!

echo "🎨 Robot Description Publisherを起動中..."
ros2 param set /robot_state_publisher robot_description "$(xacro /home/ikuo/s1_ws/src/robomaster_ros_reference/robomaster_description/urdf/robomaster_s1.urdf.xacro)" &
XACRO_PID=$!

echo "🎨 Robot State Publisherを起動中..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/ikuo/s1_ws/src/robomaster_ros_reference/robomaster_description/urdf/robomaster_s1.urdf.xacro)" &
ROBOT_STATE_PID=$!

echo "🔧 Joint State Publisherを起動中..."  
ros2 run joint_state_publisher joint_state_publisher &
JOINT_STATE_PID=$!

# 初期化を待つ
echo "⏳ すべてのノードの初期化を待機中..."
sleep 5

echo ""
echo "✅ すべてのノードが準備完了しました！"
echo ""
echo "🎮 RViz付きテレオペレーション開始"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "🎯 RVizを起動中... (ロボットモデルが表示されます)"
rviz2 &
RVIZ_PID=$!

sleep 3

echo ""
echo "📋 操作方法:"
echo "   1. RVizでロボットの状態を確認"
echo "   2. 以下のキーでロボットを操作:"
echo ""
echo "   i    k    o       前進・停止・後退"
echo "   j    ,    l       左回転・停止・右回転"  
echo "   u         .       左前・右後"
echo "   m         /       左後・右前"
echo ""
echo "   q/z : 速度増減"
echo "   w/x : 回転速度増減"
echo "   スペース: 緊急停止"
echo "   Ctrl+C : 終了"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# キーボードteleopを開始
echo "🎯 キーボード制御開始..."
trap 'echo ""; echo "🛑 テレオペレーション終了中..."; kill $DRIVER_PID $ROBOT_STATE_PID $JOINT_STATE_PID $RVIZ_PID $XACRO_PID 2>/dev/null; exit 0' INT

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel

# 終了処理
echo ""
echo "🛑 すべてのノードを停止中..."
kill $DRIVER_PID $ROBOT_STATE_PID $JOINT_STATE_PID $RVIZ_PID $XACRO_PID 2>/dev/null || true
wait $DRIVER_PID $ROBOT_STATE_PID $JOINT_STATE_PID $RVIZ_PID $XACRO_PID 2>/dev/null || true

echo "✅ ビジュアルテレオペレーション終了"
