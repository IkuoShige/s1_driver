#!/bin/bash

# RoboMaster S1 Teleoperation Menu
# 複数のテレオペレーション方法を選択できるメニュー

echo "=========================================="
echo "   RoboMaster S1 Teleoperation Menu"
echo "=========================================="
echo ""
echo "利用可能なテレオペレーション方法:"
echo ""
echo "1) キーボード制御"
echo "   - キーボードでロボットを直接制御"
echo "   - 軽量で高速応答"
echo ""
echo "2) ジョイスティック制御"  
echo "   - USBゲームパッドでの操作"
echo "   - より直感的な操作感"
echo ""
echo "3) RViz可視化付き制御"
echo "   - ロボットモデルを見ながら操作"
echo "   - センサーデータの可視化"
echo ""
echo "4) 基本テスト実行"
echo "   - システム動作確認"
echo "   - 各機能のテスト"
echo ""
echo "5) ジョイスティックデバッグ"
echo "   - ジョイスティック動作確認"
echo "   - 詳細な診断情報"
echo ""
echo "6) 終了"
echo ""

while true; do
    read -p "選択してください (1-6): " choice
    
    case $choice in
        1)
            echo ""
            echo "🎮 キーボードテレオペレーションを開始します..."
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            /home/ikuo/s1_ws/src/s1_driver/scripts/teleop_s1.sh
            break
            ;;
        2)
            echo ""
            echo "🎮 ジョイスティックテレオペレーションを開始します..."
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            /home/ikuo/s1_ws/src/s1_driver/scripts/teleop_joystick_s1.sh
            break
            ;;
        3)
            echo ""
            echo "🎨 RViz可視化付きテレオペレーションを開始します..."
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            /home/ikuo/s1_ws/src/s1_driver/scripts/teleop_visual_s1.sh
            break
            ;;
        4)
            echo ""
            echo "🔧 基本統合テストを実行します..."
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            /home/ikuo/s1_ws/src/s1_driver/scripts/test_integration.sh
            break
            ;;
        5)
            echo ""
            echo "🔍 ジョイスティックデバッグを開始します..."
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            /home/ikuo/s1_ws/src/s1_driver/scripts/debug_joystick.sh
            break
            ;;
        6)
            echo ""
            echo "👋 終了します"
            exit 0
            ;;
        *)
            echo "❌ 無効な選択です。1-6の数字を入力してください。"
            ;;
    esac
done
