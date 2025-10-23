#!/usr/bin/env bash
set -euo pipefail

CAN_IFACE=${CAN_IFACE:-vcan0}
BUILD_DIR=${BUILD_DIR:-build}
CMD_PORT=${CMD_PORT:-47001}
STATE_PORT=${STATE_PORT:-47002}

function ensure_vcan() {
  if ip link show "$CAN_IFACE" > /dev/null 2>&1; then
    return
  fi
  if ! command -v sudo > /dev/null 2>&1; then
    echo "sudo is required to create $CAN_IFACE" >&2
    exit 1
  fi
  echo "Creating $CAN_IFACE (requires sudo)"
  sudo modprobe vcan
  sudo ip link add dev "$CAN_IFACE" type vcan
  sudo ip link set up "$CAN_IFACE"
}

ensure_vcan

FSAI_RUN="$BUILD_DIR/fsai_run"
SVCU_RUN="$BUILD_DIR/svcu_run"

if [[ ! -x "$FSAI_RUN" || ! -x "$SVCU_RUN" ]]; then
  echo "Expected executables in $BUILD_DIR. Build the project before running." >&2
  exit 1
fi

trap 'kill 0' INT TERM

"$SVCU_RUN" --can-if "$CAN_IFACE" --cmd-port "$CMD_PORT" --state-port "$STATE_PORT" &
SVCU_PID=$!

echo "S-VCU started (pid=$SVCU_PID)"

"$FSAI_RUN" --can-if "$CAN_IFACE" --cmd-port "$CMD_PORT" --state-port "$STATE_PORT"

wait "$SVCU_PID"
