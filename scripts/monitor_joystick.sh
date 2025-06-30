#!/bin/bash

# Real-time Joystick Monitor for RoboMaster S1
# ジョイスティックデータをリアルタイムで監視

echo "=== リアルタイムジョイスティック監視 ==="
echo ""

# ROS2環境のセットアップ
source /home/ikuo/s1_ws/install/setup.bash

# ジョイスティックの接続確認
if [ ! -e /dev/input/js0 ]; then
    echo "❌ ジョイスティックが検出されません"
    exit 1
fi

echo "✅ ジョイスティック検出: /dev/input/js0"
echo ""

echo "🤖 S1ドライバーを起動中..."
ros2 run s1_driver s1_driver_node &
DRIVER_PID=$!

echo "🎮 Joyノードを起動中..."
ros2 run joy joy_node --ros-args -p device_id:=0 &
JOY_PID=$!

echo "🎯 Teleop_twist_joyノードを起動中..."
ros2 run teleop_twist_joy teleop_node --ros-args \
  --params-file /home/ikuo/s1_ws/src/s1_driver/config/joystick_teleop.yaml \
  --remap cmd_vel:=/cmd_vel &
TELEOP_PID=$!

sleep 3

echo ""
echo "📊 リアルタイム監視開始"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📋 操作方法:"
echo "   左スティック上下: 前後移動 (axes[1])"
echo "   右スティック左右: 回転 (axes[2])"
echo "   ボタンA: 有効化 (buttons[0]) ← これを押しながら操作！"
echo "   ボタンB: ターボモード (buttons[1])"
echo ""
echo "   'j' : Joy topicを表示"
echo "   'c' : cmd_vel topicを表示"
echo "   'q' : 終了"
echo ""

trap 'echo ""; echo "🛑 監視終了中..."; kill $DRIVER_PID $JOY_PID $TELEOP_PID 2>/dev/null; exit 0' INT

# インタラクティブ監視ループ
while true; do
    echo -n "コマンド [j/c/q]: "
    read -n 1 cmd
    echo ""
    
    case $cmd in
        j|J)
            echo "📊 Joy topic (3秒間監視) - ジョイスティックを動かしてみてください:"
            echo "----------------------------------------"
            timeout 3s ros2 topic echo /joy | head -20
            echo "----------------------------------------"
            ;;
        c|C)
            echo "🚗 cmd_vel topic (3秒間監視) - ボタンAを押しながらスティックを動かしてください:"
            echo "----------------------------------------"
            timeout 3s ros2 topic echo /cmd_vel | head -10
            echo "----------------------------------------"
            ;;
        q|Q)
            break
            ;;
        *)
            echo "無効なコマンドです。j, c, または q を入力してください。"
            ;;
    esac
    echo ""
done

# 終了処理
echo "🛑 すべてのノードを停止中..."
kill $DRIVER_PID $JOY_PID $TELEOP_PID 2>/dev/null || true
wait $DRIVER_PID $JOY_PID $TELEOP_PID 2>/dev/null || true

echo "✅ 監視終了"
