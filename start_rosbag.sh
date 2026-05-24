#!/usr/bin/env bash
if [ -z "${BASH_VERSION:-}" ]; then
    exec bash "$0" "$@"
fi

set -Eeo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

SETUP_FILE="$WORKSPACE_DIR/install/setup.bash"
if [[ ! -f "$SETUP_FILE" ]]; then
    echo "未找到 $SETUP_FILE，请先在工程根目录完成 colcon build。"
    exit 1
fi

# shellcheck disable=SC1090
source "$SETUP_FILE"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
    echo "未找到 ros2 命令，请检查 ROS2 环境。"
    exit 1
fi

DEFAULT_ROSBAG_FILE="/home/hwx/rosbag/rosbag_1.db3"
ROSBAG_FILE="${1:-${ROSBAG_FILE:-$DEFAULT_ROSBAG_FILE}}"
LOG_DIR="${LOG_DIR:-/tmp/radar2026_rosbag_$(date +%Y%m%d_%H%M%S)}"
START_DELAY="${START_DELAY:-2}"
SKIP_CALIBRATION="${SKIP_CALIBRATION:-0}"
DRY_RUN="${DRY_RUN:-0}"

if [[ "$ROSBAG_FILE" != /* ]]; then
    ROSBAG_FILE="$WORKSPACE_DIR/$ROSBAG_FILE"
fi

if [[ "$DRY_RUN" != "1" && ! -e "$ROSBAG_FILE" ]]; then
    echo "未找到 rosbag 文件或目录: $ROSBAG_FILE"
    echo "用法: ./start_rosbag.sh /path/to/rosbag.db3"
    echo "也可以设置: ROSBAG_FILE=/path/to/rosbag.db3 ./start_rosbag.sh"
    exit 1
fi

mkdir -p "$LOG_DIR"

PIDS=()
NAMES=()
LAST_PID=""

register_process() {
    local name="$1"
    local pid="$2"
    PIDS+=("$pid")
    NAMES+=("$name")
    LAST_PID="$pid"
}

process_group_exists() {
    local pid="$1"
    kill -0 -- "-$pid" >/dev/null 2>&1
}

unregister_process() {
    local target_pid="$1"
    local next_pids=()
    local next_names=()
    local index

    for index in "${!PIDS[@]}"; do
        if [[ "${PIDS[$index]}" != "$target_pid" ]]; then
            next_pids+=("${PIDS[$index]}")
            next_names+=("${NAMES[$index]}")
        fi
    done

    PIDS=("${next_pids[@]}")
    NAMES=("${next_names[@]}")
}

stop_one() {
    local pid="$1"
    local name="$2"

    if process_group_exists "$pid"; then
        echo "关闭 $name ..."
        kill -INT -- "-$pid" >/dev/null 2>&1 || true
        sleep 2
    fi

    if process_group_exists "$pid"; then
        kill -TERM -- "-$pid" >/dev/null 2>&1 || true
        sleep 1
    fi

    if process_group_exists "$pid"; then
        kill -KILL -- "-$pid" >/dev/null 2>&1 || true
    fi

    wait "$pid" >/dev/null 2>&1 || true
    unregister_process "$pid"
}

stop_all() {
    local index
    local pid
    local name

    if ((${#PIDS[@]} == 0)); then
        return
    fi

    echo
    echo "正在关闭本脚本启动的节点..."
    for index in "${!PIDS[@]}"; do
        pid="${PIDS[$index]}"
        name="${NAMES[$index]}"
        if process_group_exists "$pid"; then
            echo "  SIGINT $name (PGID $pid)"
            kill -INT -- "-$pid" >/dev/null 2>&1 || true
        fi
    done

    sleep 2

    for index in "${!PIDS[@]}"; do
        pid="${PIDS[$index]}"
        name="${NAMES[$index]}"
        if process_group_exists "$pid"; then
            echo "  SIGTERM $name (PGID $pid)"
            kill -TERM -- "-$pid" >/dev/null 2>&1 || true
        fi
    done

    sleep 1

    for index in "${!PIDS[@]}"; do
        pid="${PIDS[$index]}"
        name="${NAMES[$index]}"
        if process_group_exists "$pid"; then
            echo "  SIGKILL $name (PGID $pid)"
            kill -KILL -- "-$pid" >/dev/null 2>&1 || true
        fi
    done

    for pid in "${PIDS[@]}"; do
        wait "$pid" >/dev/null 2>&1 || true
    done
}

cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM
    stop_all
    exit "$exit_code"
}

run_background() {
    local name="$1"
    local wait_after="$2"
    shift 2

    local log_file="$LOG_DIR/${name}.log"

    echo "[$(date +%H:%M:%S)] 启动 $name"
    echo "  命令: $*"
    echo "  日志: $log_file"

    if [[ "$DRY_RUN" == "1" ]]; then
        return
    fi

    setsid "$@" >"$log_file" 2>&1 &
    register_process "$name" "$!"

    sleep "$wait_after"

    if ! process_group_exists "$LAST_PID"; then
        local status=0
        wait "$LAST_PID" >/dev/null 2>&1 || status=$?
        echo
        echo "$name 启动后已退出，退出码: $status"
        if [[ -f "$log_file" ]]; then
            echo "日志最后 80 行:"
            tail -n 80 "$log_file"
        fi
        exit "$status"
    fi
}

trap cleanup EXIT INT TERM

echo "Radar2026 rosbag 一键启动"
echo "工作目录: $WORKSPACE_DIR"
echo "rosbag: $ROSBAG_FILE"
echo "日志目录: $LOG_DIR"
echo

if [[ "$SKIP_CALIBRATION" == "1" ]]; then
    echo "[01] 跳过 rosbag 五点标定。"
else
    run_background "01_calib_rosbag" "$START_DELAY" \
        ros2 launch tdt_vision calib_rosbag.launch.py "rosbag_file:=$ROSBAG_FILE"

    if [[ "$DRY_RUN" == "1" ]]; then
        echo "[01] DRY_RUN 模式，不等待标定交互。"
    else
        CALIBRATION_PID="$LAST_PID"
        echo
        echo "[01] rosbag 五点标定已启动。"
        echo "     在标定窗口按 Enter 开始，依次选满 5 个点并保存外参。"
        read -r -p "     标定完成后回到此终端按 Enter，脚本会关闭标定节点并继续启动回放: "
        stop_one "$CALIBRATION_PID" "01_calib_rosbag"
    fi

    if [[ ! -s "$WORKSPACE_DIR/config/out_matrix.yaml" ]]; then
        echo "警告: 未检测到 config/out_matrix.yaml，后续 radar 解算可能无法正常工作。"
    fi

    sleep "$START_DELAY"
fi

run_background "02_run_rosbag" "$START_DELAY" \
    ros2 launch tdt_vision run_rosbag.launch.py "rosbag_file:=$ROSBAG_FILE"

echo
echo "rosbag 回放主流程已启动。"

if [[ "$DRY_RUN" == "1" ]]; then
    echo "DRY_RUN 模式未实际启动节点。"
    exit 0
fi

echo "按 Ctrl+C 可统一关闭这些进程。"

while true; do
    sleep 3600
done
