#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
FAKE_BIN=$(mktemp -d)
CALL_LOG=$(mktemp)

cleanup() {
    rm -rf "$FAKE_BIN"
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
    has-session) exit "${FAKE_TMUX_HAS_SESSION_EXIT:-1}" ;;
    list-panes) printf '%%0\n%%1\n%%2\n' ;;
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
exit 0
EOF
chmod +x "$FAKE_BIN/tmux" "$FAKE_BIN/gnome-terminal" "$FAKE_BIN/sleep"

FAKE_CALL_LOG="$CALL_LOG" PATH="$FAKE_BIN:$PATH" \
    bash "$PROJECT_ROOT/scripts/start_navigation_height_obstacle.sh" >/dev/null

grep -Fq -- 'tmux new-session -d -s mppi_nav_height_obstacle -n main' "$CALL_LOG"
grep -Fq -- 'nav2_params_height_obstacle.yaml' "$CALL_LOG"
grep -Fq -- 'python3\ keep_zero.py' "$CALL_LOG"
grep -Fq -- 'initial_pose_tf.py' "$CALL_LOG"

: > "$CALL_LOG"
FAKE_CALL_LOG="$CALL_LOG" FAKE_TMUX_HAS_SESSION_EXIT=0 PATH="$FAKE_BIN:$PATH" \
    bash "$PROJECT_ROOT/scripts/stop_navigation_height_obstacle.sh" >/dev/null
grep -Fq -- 'tmux kill-session -t mppi_nav_height_obstacle' "$CALL_LOG"

printf 'PASS: HeightObstacle navigation start/stop scripts\n'
