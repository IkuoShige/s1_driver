#!/bin/bash

# RoboMaster S1 Joystick Debug Script
# ジョイスティックの動作確認とデバッグ用スクリプト

echo "=== RoboMaster S1 Joystick Debug ==="
echo ""

# ROS2環境のセットアップ
source /home/ikuo/s1_ws/install/setup.bash

# ジョイスティックの接続確認
echo "🎮 ジョイスティック接続確認..."
if [ ! -e /dev/input/js0 ]; then
    echo "❌ ジョイスティックが検出されません"
    echo "   USBジョイスティックを接続してから再実行してください"
    exit 1
fi

echo "✅ ジョイスティックが検出されました: /dev/input/js0"
echo ""

# ジョイスティック情報表示
echo "📋 ジョイスティック情報:"
jstest --print /dev/input/js0 2>/dev/null | head -3 || echo "jstestが利用できません"
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

echo "🎮 Joyノードテスト開始..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# joyノードのみを起動してテスト
echo "1. Joyノード単体テスト（5秒間）..."
ros2 run joy joy_node --ros-args -p device_id:=0 &
JOY_PID=$!

sleep 2
echo "   📊 Joy topicの内容をチェック中..."
timeout 3s ros2 topic echo /joy --once 2>/dev/null && \
  echo "   ✅ Joyトピック正常動作" || echo "   ❌ Joyトピック応答なし"

kill $JOY_PID 2>/dev/null
wait $JOY_PID 2>/dev/null || true

echo ""
echo "2. 手動コマンドテスト..."
echo "   📤 手動でcmd_velコマンド送信中..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}" \
  > /dev/null 2>&1
echo "   ✅ 手動コマンド送信完了"

sleep 1

echo "   📤 停止コマンド送信中..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  > /dev/null 2>&1

echo ""
echo "3. 完全なジョイスティック制御テスト..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📋 ジョイスティック操作方法:"
echo "   左スティック上下: 前後移動"
echo "   右スティック左右: 回転"
echo "   ボタン1 (A): 有効化ボタン（押しながら操作）"
echo "   ボタン2 (B): ターボモード"
echo ""
echo "⚠️  重要: ボタン1 (Aボタン) を押しながらスティックを動かしてください"
echo ""

trap 'echo ""; echo "🛑 デバッグテスト終了中..."; kill $DRIVER_PID $JOY_PID $TELEOP_PID 2>/dev/null; exit 0' INT

# 完全なジョイスティック制御を開始
echo "🎯 ジョイスティック制御開始..."

# joyノードを起動
ros2 run joy joy_node --ros-args -p device_id:=0 &
JOY_PID=$!

sleep 2

# teleop_twist_joyノードを起動
ros2 run teleop_twist_joy teleop_node --ros-args \
  --params-file /home/ikuo/s1_ws/src/s1_driver/config/joystick_teleop.yaml \
  --remap cmd_vel:=/cmd_vel &
TELEOP_PID=$!

echo "🟢 ジョイスティック制御が開始されました"
echo "   ジョイスティックを操作してください（ボタン1を押しながら）"
echo "   Ctrl+C で終了"
echo ""

# トピック監視
echo "📊 制御状況の監視開始..."
while true; do
    echo -n "."
    sleep 2
done

# 終了処理（Ctrl+Cで実行される）
echo ""
echo "🛑 すべてのノードを停止中..."
kill $DRIVER_PID $JOY_PID $TELEOP_PID 2>/dev/null || true
wait $DRIVER_PID $JOY_PID $TELEOP_PID 2>/dev/null || true

echo "✅ ジョイスティックデバッグ終了"
