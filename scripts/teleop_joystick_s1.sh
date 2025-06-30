#!/bin/bash

# RoboMaster S1 Joystick Teleoperation Script
# ジョイスティックでロボットを遠隔操作するスクリプト

echo "=== RoboMaster S1 Joystick Teleoperation ==="
echo ""

# ROS2環境のセットアップ
source /home/ikuo/s1_ws/install/setup.bash

# ジョイスティックの接続確認
if [ ! -e /dev/input/js0 ]; then
    echo "❌ ジョイスティックが接続されていません"
    echo "   USBジョイスティックを接続してから再実行してください"
    exit 1
fi

echo "🎮 ジョイスティックが検出されました: /dev/input/js0"
echo ""

echo "🤖 S1ドライバーを起動中..."
ros2 run s1_driver s1_driver_node &
DRIVER_PID=$!

# ドライバーの初期化を待つ
echo "⏳ ドライバーの初期化を待機中..."
sleep 4

echo ""
echo "✅ S1ドライバーが準備完了しました！"
echo ""
echo "🎮 ジョイスティックテレオペレーション開始"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📋 ジョイスティック操作方法:"
echo "   左スティック: 前後移動"
echo "   右スティック: 回転"
echo "   ボタン1 (通常Aボタン): 緊急停止"
echo "   ボタン2 (通常Bボタン): 速度リセット"
echo ""
echo "   Ctrl+C : 終了"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# joy nodeとteleop_twist_joyを同時に起動
echo "🎯 ジョイスティック制御開始..."
trap 'echo ""; echo "🛑 テレオペレーション終了中..."; kill $DRIVER_PID $JOY_PID $TELEOP_PID 2>/dev/null; exit 0' INT

# joyノードを起動（デバイス指定）
ros2 run joy joy_node --ros-args -p device_id:=0 &
JOY_PID=$!

sleep 2

# teleop_twist_joyノードを設定ファイル付きで起動
ros2 run teleop_twist_joy teleop_node --ros-args \
  --params-file /home/ikuo/s1_ws/src/s1_driver/config/joystick_teleop.yaml \
  --remap cmd_vel:=/cmd_vel &
TELEOP_PID=$!

echo "🟢 ジョイスティック制御が開始されました"
echo "   ジョイスティックでロボットを操作してください"
echo ""

# 無限ループで待機
while true; do
    sleep 1
done

# 終了処理（Ctrl+Cで実行される）
echo ""
echo "🛑 すべてのノードを停止中..."
kill $DRIVER_PID $JOY_PID $TELEOP_PID 2>/dev/null || true
wait $DRIVER_PID $JOY_PID $TELEOP_PID 2>/dev/null || true

echo "✅ ジョイスティックテレオペレーション終了"
