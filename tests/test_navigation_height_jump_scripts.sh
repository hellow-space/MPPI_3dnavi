#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
FAKE_BIN=$(mktemp -d)
CALL_LOG=$(mktemp)
SPACED_ROOT=$(mktemp -d "/tmp/mppi height jump test.XXXXXX")

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
        printf '%%0\n%%1\n%%2\n'
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

assert_log_not_contains() {
    local unexpected=$1
    if grep -Fq -- "$unexpected" "$CALL_LOG"; then
        printf 'FAIL: expected command log not to contain: %s\n' "$unexpected" >&2
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
    bash "$PROJECT_ROOT/scripts/start_navigation_height_jump.sh" > /dev/null

assert_log_contains 'tmux new-session -d -s mppi_nav_height_jump -n main'
assert_log_count 2 'tmux split-window'
printf -v expected_navigation '%q' \
    "ros2 launch fishbot_navigation2 navigation2.launch.py params_file:=$PROJECT_ROOT/src/fishbot_navigation2/config/nav2_params_height_jump.yaml"
printf -v expected_initial_pose '%q' \
    "python3 $PROJECT_ROOT/scripts/initial_pose_tf.py"
assert_log_contains \
    "tmux send-keys -t mppi_nav_height_jump:0.0 $expected_navigation C-m"
assert_log_contains \
    'tmux send-keys -t mppi_nav_height_jump:0.1 python3\ keep_zero.py C-m'
assert_log_contains \
    "tmux send-keys -t mppi_nav_height_jump:0.2 $expected_initial_pose C-m"
assert_log_contains 'gnome-terminal --window --title=MPPI\ Navigation\ HeightJump'

: > "$CALL_LOG"

mkdir -p "$SPACED_ROOT/scripts"
ln -s "$PROJECT_ROOT/scripts/start_navigation_height_jump.sh" \
    "$SPACED_ROOT/scripts/start_navigation_height_jump.sh"

FAKE_CALL_LOG="$CALL_LOG" PATH="$FAKE_BIN:$PATH" \
    bash "$SPACED_ROOT/scripts/start_navigation_height_jump.sh" > /dev/null

printf -v quoted_spaced_root '%q' "$SPACED_ROOT"
printf -v expected_spaced_cd '%q' "cd $quoted_spaced_root"
printf -v expected_spaced_tf '%q' \
    "python3 $quoted_spaced_root/scripts/initial_pose_tf.py"
assert_log_contains \
    "tmux send-keys -t mppi_nav_height_jump:0.0 $expected_spaced_cd C-m"
assert_log_contains "$expected_spaced_tf"

: > "$CALL_LOG"

FAKE_CALL_LOG="$CALL_LOG" FAKE_TMUX_HAS_SESSION_EXIT=0 PATH="$FAKE_BIN:$PATH" \
    bash "$PROJECT_ROOT/scripts/start_navigation_height_jump.sh" > /dev/null

assert_log_contains \
    'tmux list-panes -s -t mppi_nav_height_jump -F \#\{pane_id\}'
assert_log_count 3 'tmux send-keys -t %'
assert_log_contains 'sleep 2'
assert_log_contains 'tmux kill-session -t mppi_nav_height_jump'
assert_log_contains 'tmux new-session -d -s mppi_nav_height_jump -n main'

: > "$CALL_LOG"

start_failure_output=$(mktemp)
if FAKE_CALL_LOG="$CALL_LOG" FAKE_TMUX_HAS_SESSION_EXIT=0 \
    FAKE_TMUX_KILL_SESSION_EXIT=1 PATH="$FAKE_BIN:$PATH" \
    bash "$PROJECT_ROOT/scripts/start_navigation_height_jump.sh" \
        > /dev/null 2> "$start_failure_output"; then
    printf 'FAIL: repeated start should fail when the old session cannot be closed\n' >&2
    exit 1
fi
if ! grep -Fq -- '关闭失败' "$start_failure_output"; then
    printf 'FAIL: repeated start failure should explain that shutdown failed\n' >&2
    cat "$start_failure_output" >&2
    exit 1
fi
rm -f "$start_failure_output"
assert_log_not_contains 'tmux new-session'

: > "$CALL_LOG"

FAKE_CALL_LOG="$CALL_LOG" FAKE_TMUX_HAS_SESSION_EXIT=0 PATH="$FAKE_BIN:$PATH" \
    bash "$PROJECT_ROOT/scripts/stop_navigation_height_jump.sh" > /dev/null

assert_log_contains \
    'tmux list-panes -s -t mppi_nav_height_jump -F \#\{pane_id\}'
assert_log_count 3 'tmux send-keys -t %'
assert_log_contains 'tmux kill-session -t mppi_nav_height_jump'

: > "$CALL_LOG"

FAKE_CALL_LOG="$CALL_LOG" FAKE_TMUX_HAS_SESSION_EXIT=0 \
    FAKE_TMUX_SEND_KEYS_EXIT=1 PATH="$FAKE_BIN:$PATH" \
    bash "$PROJECT_ROOT/scripts/stop_navigation_height_jump.sh" > /dev/null

assert_log_contains 'tmux kill-session -t mppi_nav_height_jump'

: > "$CALL_LOG"

stop_output=$(mktemp)
FAKE_CALL_LOG="$CALL_LOG" PATH="$FAKE_BIN:$PATH" \
    bash "$PROJECT_ROOT/scripts/stop_navigation_height_jump.sh" > "$stop_output"

if ! grep -Fq -- '当前没有运行' "$stop_output"; then
    printf 'FAIL: stopping an absent session should report that it is not running\n' >&2
    cat "$stop_output" >&2
    exit 1
fi
rm -f "$stop_output"
assert_log_not_contains 'tmux kill-session'

printf 'PASS: HeightJump navigation start/stop scripts\n'
