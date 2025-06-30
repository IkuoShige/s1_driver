#!/bin/bash

# RoboMaster S1 Teleoperation Script
# キーボードでロボットを遠隔操作するスクリプト

echo "=== RoboMaster S1 Teleoperation ==="
echo ""

# ROS2環境のセットアップ
source /home/ikuo/s1_ws/install/setup.bash

echo "🤖 S1ドライバーを起動中..."
ros2 run s1_driver s1_driver_node &
DRIVER_PID=$!

# ドライバーの初期化を待つ
echo "⏳ ドライバーの初期化を待機中..."
sleep 4

echo ""
echo "✅ S1ドライバーが準備完了しました！"
echo ""
echo "🎮 キーボードテレオペレーション開始"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📋 操作方法:"
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
trap 'echo ""; echo "🛑 テレオペレーション終了中..."; kill $DRIVER_PID 2>/dev/null; exit 0' INT

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel

# 終了処理
echo ""
echo "🛑 S1ドライバーを停止中..."
kill $DRIVER_PID 2>/dev/null || true
wait $DRIVER_PID 2>/dev/null || true

echo "✅ テレオペレーション終了"
