#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
FAKE_BIN=$(mktemp -d)
CALL_LOG=$(mktemp)
SPACED_ROOT=$(mktemp -d "/tmp/mppi nav test.XXXXXX")

cleanup() {
    rm -rf "$FAKE_BIN"
    rm -rf "$SPACED_ROOT"
    rm -f "$CALL_LOG"
}
trap cleanup EXIT

cat > "$FAKE_BIN/tmux" <<'EOF'
#!/usr/bin/env bash
{
    printf 'tmux'
    printf ' %q' "$@"
    printf '\n'
} >> "$FAKE_CALL_LOG"

case "${1:-}" in
    has-session)
        exit "${FAKE_TMUX_HAS_SESSION_EXIT:-1}"
        ;;
    send-keys)
        exit "${FAKE_TMUX_SEND_KEYS_EXIT:-0}"
        ;;
    kill-session)
        exit "${FAKE_TMUX_KILL_SESSION_EXIT:-0}"
        ;;
    list-panes)
        printf '%%0\n%%1\n%%2\n%%3\n'
        ;;
esac
EOF

cat > "$FAKE_BIN/gnome-terminal" <<'EOF'
#!/usr/bin/env bash
{
    printf 'gnome-terminal'
    printf ' %q' "$@"
    printf '\n'
} >> "$FAKE_CALL_LOG"
EOF

cat > "$FAKE_BIN/sleep" <<'EOF'
#!/usr/bin/env bash
{
    printf 'sleep'
    printf ' %q' "$@"
    printf '\n'
} >> "$FAKE_CALL_LOG"
EOF

chmod +x "$FAKE_BIN/tmux" "$FAKE_BIN/gnome-terminal" "$FAKE_BIN/sleep"

assert_log_contains() {
    local expected=$1
    if ! grep -Fq -- "$expected" "$CALL_LOG"; then
        printf 'FAIL: expected command log to contain: %s\n' "$expected" >&2
        printf '%s\n' '--- command log ---' >&2
        cat "$CALL_LOG" >&2
        exit 1
    fi
}

assert_log_count() {
    local expected_count=$1
    local pattern=$2
    local actual_count
    actual_count=$(grep -Fc -- "$pattern" "$CALL_LOG" || true)
    if [[ "$actual_count" != "$expected_count" ]]; then
        printf 'FAIL: expected %s occurrences of %s, got %s\n' \
            "$expected_count" "$pattern" "$actual_count" >&2
        cat "$CALL_LOG" >&2
        exit 1
    fi
}

FAKE_CALL_LOG="$CALL_LOG" PATH="$FAKE_BIN:$PATH" \
    bash "$PROJECT_ROOT/scripts/start_navigation_with_tf.sh" > /dev/null

assert_log_contains 'tmux new-session -d -s mppi_nav_with_tf -n main'
assert_log_count 3 'tmux split-window'
printf -v expected_navigation '%q' \
    "ros2 launch fishbot_navigation2 navigation2.launch.py params_file:=$PROJECT_ROOT/src/fishbot_navigation2/config/2d_params.yaml"
printf -v expected_initial_pose '%q' \
    "python3 $PROJECT_ROOT/scripts/initial_pose_tf.py"
printf -v expected_scan_level '%q' \
    "python3 $PROJECT_ROOT/scripts/scan_level_tf.py"
assert_log_contains "$expected_navigation"
assert_log_contains 'python3\ keep_zero.py'
assert_log_contains "$expected_initial_pose"
assert_log_contains "$expected_scan_level"
assert_log_contains 'gnome-terminal --window --title=MPPI\ Navigation\ +\ TF'

: > "$CALL_LOG"

mkdir -p "$SPACED_ROOT/scripts"
ln -s "$PROJECT_ROOT/scripts/start_navigation_with_tf.sh" \
    "$SPACED_ROOT/scripts/start_navigation_with_tf.sh"

FAKE_CALL_LOG="$CALL_LOG" PATH="$FAKE_BIN:$PATH" \
    bash "$SPACED_ROOT/scripts/start_navigation_with_tf.sh" > /dev/null

printf -v quoted_spaced_root '%q' "$SPACED_ROOT"
printf -v expected_spaced_cd '%q' "cd $quoted_spaced_root"
printf -v expected_spaced_tf '%q' \
    "python3 $quoted_spaced_root/scripts/initial_pose_tf.py"
assert_log_contains "tmux send-keys -t mppi_nav_with_tf:0.0 $expected_spaced_cd C-m"
assert_log_contains "$expected_spaced_tf"

: > "$CALL_LOG"

FAKE_CALL_LOG="$CALL_LOG" FAKE_TMUX_HAS_SESSION_EXIT=0 PATH="$FAKE_BIN:$PATH" \
    bash "$PROJECT_ROOT/scripts/stop_navigation_with_tf.sh" > /dev/null

assert_log_count 4 'tmux send-keys -t %'
assert_log_contains 'tmux kill-session -t mppi_nav_with_tf'

: > "$CALL_LOG"

FAKE_CALL_LOG="$CALL_LOG" FAKE_TMUX_HAS_SESSION_EXIT=0 \
    FAKE_TMUX_SEND_KEYS_EXIT=1 PATH="$FAKE_BIN:$PATH" \
    bash "$PROJECT_ROOT/scripts/stop_navigation_with_tf.sh" > /dev/null

assert_log_contains 'tmux kill-session -t mppi_nav_with_tf'

if close_output=$(FAKE_CALL_LOG="$CALL_LOG" FAKE_TMUX_HAS_SESSION_EXIT=0 \
    FAKE_TMUX_KILL_SESSION_EXIT=1 PATH="$FAKE_BIN:$PATH" \
    bash "$PROJECT_ROOT/scripts/stop_navigation_with_tf.sh" 2>&1); then
    printf 'FAIL: stop script reported success while the session remained\n' >&2
    exit 1
fi

if [[ "$close_output" != *'关闭失败'* ]]; then
    printf 'FAIL: stop script did not explain that closing failed\n' >&2
    exit 1
fi

printf 'PASS: navigation with TF start/stop scripts\n'
