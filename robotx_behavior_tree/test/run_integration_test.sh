set -eo pipefail
cleanup() {
  echo ">>> Cleaning up launch process group (PGID: $LAUNCH_PGID)..."
  if [ -n "$LAUNCH_PGID" ]; then
    kill -SIGTERM -- "-$LAUNCH_PGID" 2>/dev/null
  fi
}
trap cleanup EXIT

BT_PKG_SHARE=$(ros2 pkg prefix robotx_behavior_tree)/share/robotx_behavior_tree
CONFIG_FILE="/config/go_around_object.yaml"
echo ">>> Starting GTest executable in the background..."
./test_go_around_object &
GTEST_PID=$!
echo ">>> GTest executable started with PID $GTEST_PID."
echo ">>> Waiting 5 seconds for test node to initialize..."
sleep 5

set -m
echo ">>> Launching Simulation in the background..."
ros2 launch navi_sim with_planner.launch.py \
  behavior_config_filepath:="$CONFIG_FILE" &
LAUNCH_PID=$! 
LAUNCH_PGID=$(ps -o pgid= -p "$LAUNCH_PID" | tr -d ' ')
set +m

echo ">>> Simulation launched with PID $LAUNCH_PID in process group $LAUNCH_PGID."
echo ">>> Waiting for GTest executable to complete..."
wait "$GTEST_PID"
exit $?
