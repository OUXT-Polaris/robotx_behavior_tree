#!/bin/bash

# -e: コマンドがエラーになったら直ちにスクリプトを終了する
# -o pipefail: パイプの途中でコマンドが失敗した場合もエラーとする
set -eo pipefail

# スクリプト終了時にバックグラウンドプロセスをクリーンアップするための関数
cleanup() {
  echo ">>> Cleaning up launch process group (PGID: $LAUNCH_PGID)..."
  # LAUNCH_PGIDが設定されていれば、そのプロセスグループ全体に終了シグナルを送る
  if [ -n "$LAUNCH_PGID" ]; then
    # プロセスグループIDの前にハイフンをつけることで、グループ全体にシグナルを送る
    kill -SIGTERM -- "-$LAUNCH_PGID" 2>/dev/null
  fi
}

# スクリプトが終了する際(EXIT)に必ずcleanup関数を呼び出すように設定
trap cleanup EXIT

# 必要なパッケージのパスを動的に解決
BT_PKG_SHARE=$(ros2 pkg prefix robotx_behavior_tree)/share/robotx_behavior_tree
CONFIG_FILE="$BT_PKG_SHARE/config/go_around_object.yaml"

# --- 変更点: ここから ---

echo ">>> Starting GTest executable in the background..."
./test_go_around_object &
GTEST_PID=$!
echo ">>> GTest executable started with PID $GTEST_PID."

# テストノードが起動し、トピックの購読準備が整うまで少し待機する
echo ">>> Waiting 5 seconds for test node to initialize..."
sleep 5

# ジョブコントロールを有効にし、バックグラウンドプロセスが新しいプロセスグループを作成するようにする
set -m

echo ">>> Launching Simulation in the background..."
ros2 launch navi_sim with_planner.launch.py \
  behavior_config_filepath:="$CONFIG_FILE" &
LAUNCH_PID=$! # このPIDはPGID取得にのみ使用
# psコマンドでプロセスIDからプロセスグループID(PGID)を取得する
LAUNCH_PGID=$(ps -o pgid= -p "$LAUNCH_PID" | tr -d ' ')
set +m
echo ">>> Simulation launched with PID $LAUNCH_PID in process group $LAUNCH_PGID."

echo ">>> Waiting for GTest executable to complete..."
wait "$GTEST_PID"
exit $?
