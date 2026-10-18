#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
python3 - "$PROJECT_ROOT" <<'PY'
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(sys.argv.pop())
SESSION = "mppi_nav_draw_path"

FAKE_TMUX = '''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
args = sys.argv[1:]
with open(os.environ["FAKE_CALL_LOG"], "a") as log:
    log.write(json.dumps(["tmux", *args]) + "\\n")
state = Path(os.environ["FAKE_SESSION_STATE"])
if args[0] == "has-session":
    sys.exit(0 if state.exists() else 1)
if args[0] == "list-panes":
    print("%0\\n%1\\n%2")
elif args[0] == "kill-session":
    if os.environ.get("FAKE_KILL_FAIL") == "1": sys.exit(1)
    state.unlink(missing_ok=True)
elif args[0] == "new-session":
    state.touch()
elif args[0] == "send-keys" and args[-1] == "C-c":
    if os.environ.get("FAKE_CTRL_C_FAIL") == "1": sys.exit(1)
'''

FAKE_TERMINAL = '''#!/usr/bin/env python3
import json, os, sys
with open(os.environ["FAKE_CALL_LOG"], "a") as log:
    log.write(json.dumps(["gnome-terminal", *sys.argv[1:]]) + "\\n")
'''


class DrawPathScriptsTest(unittest.TestCase):
    def setUp(self):
        self.assertTrue((ROOT / "scripts/start_navigation_draw_path.sh").is_file(),
                        "new start script is missing")
        self.assertTrue((ROOT / "scripts/stop_navigation_draw_path.sh").is_file(),
                        "new stop script is missing")
        self.temp = tempfile.TemporaryDirectory(prefix="draw path scripts ")
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.project = self.base / "project with spaces"
        self.scripts = self.project / "scripts"
        self.config = self.project / "src/fishbot_navigation2/config"
        self.bin = self.base / "bin"
        for directory in (self.scripts, self.config, self.bin):
            directory.mkdir(parents=True)
        for name in ("start_navigation_draw_path.sh", "stop_navigation_draw_path.sh"):
            shutil.copy2(ROOT / "scripts" / name, self.scripts / name)
        for name in ("nav2_params_height_obstacle.yaml", "nav2_params_height_jump.yaml"):
            shutil.copy2(ROOT / "src/fishbot_navigation2/config" / name, self.config / name)
        for name, content in (("tmux", FAKE_TMUX), ("gnome-terminal", FAKE_TERMINAL),
                              ("sleep", "#!/usr/bin/env bash\nexit 0\n")):
            executable = self.bin / name
            executable.write_text(content)
            executable.chmod(0o755)
        self.log = self.base / "calls.jsonl"
        self.state = self.base / "session"
        self.env = dict(os.environ, PATH=f"{self.bin}:{os.environ['PATH']}",
                        FAKE_CALL_LOG=str(self.log), FAKE_SESSION_STATE=str(self.state))

    def run_script(self, name="start", *args, **env):
        return subprocess.run(["bash", str(self.scripts / f"{name}_navigation_draw_path.sh"),
                               *args], env=dict(self.env, **env), capture_output=True, text=True)

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def test_default_starts_three_panes_and_default_yaml(self):
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.calls()
        self.assertIn(["tmux", "new-session", "-d", "-s", SESSION, "-n", "main"], calls)
        self.assertEqual(sum(c[1] == "split-window" for c in calls if c[0] == "tmux"), 2)
        commands = [c[-2] for c in calls if c[0:2] == ["tmux", "send-keys"] and c[-1] == "C-m"]
        navigation = next(c for c in commands if c.startswith("ros2 launch"))
        self.assertIn("navigation_draw_path.launch.py", navigation)
        self.assertIn("nav2_params_height_obstacle.yaml", navigation)
        self.assertIn("python3 keep_zero.py", commands)
        self.assertTrue(any("initial_pose_tf.py" in c for c in commands))
        self.assertTrue(any(c[0] == "gnome-terminal" for c in calls))

    def test_existing_alternate_basename_passes_unchanged(self):
        result = self.run_script("start", "nav2_params_height_jump.yaml")
        self.assertEqual(result.returncode, 0, result.stderr)
        launch = next(c[-2] for c in self.calls() if c[0:2] == ["tmux", "send-keys"]
                      and "ros2 launch" in c[-2])
        self.assertIn("nav2_params_height_jump.yaml", launch)
        self.assertNotIn("nav2_params_height_obstacle.yaml", launch)

    def test_invalid_or_missing_yaml_never_touches_tmux(self):
        for arguments in (("missing.yaml",), ("/tmp/absolute.yaml",), ("dir/config.yaml",),
                          ("../nav2_params_height_obstacle.yaml",), ("foo..yaml",),
                          ("nav2_params_height_obstacle.yml",), ("config.txt",), ("",),
                          ("config.yaml", "extra.yaml")):
            with self.subTest(arguments=arguments):
                result = self.run_script("start", *arguments)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(self.calls(), [])

    def test_yaml_directory_is_rejected_before_tmux(self):
        (self.config / "directory.yaml").mkdir()
        self.assertNotEqual(self.run_script("start", "directory.yaml").returncode, 0)
        self.assertEqual(self.calls(), [])

    def test_repeat_start_closes_all_old_panes_before_new_session(self):
        self.state.touch()
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.calls()
        kill = calls.index(["tmux", "kill-session", "-t", SESSION])
        new = calls.index(["tmux", "new-session", "-d", "-s", SESSION, "-n", "main"])
        self.assertLess(kill, new)
        for pane in ("%0", "%1", "%2"):
            self.assertLess(calls.index(["tmux", "send-keys", "-t", pane, "C-c"]), kill)

    def test_failed_shutdown_blocks_new_start_even_if_ctrl_c_fails(self):
        self.state.touch()
        result = self.run_script(FAKE_KILL_FAIL="1", FAKE_CTRL_C_FAIL="1")
        self.assertNotEqual(result.returncode, 0)
        calls = self.calls()
        self.assertIn(["tmux", "kill-session", "-t", SESSION], calls)
        self.assertFalse(any(c[0:2] == ["tmux", "new-session"] for c in calls))

    def test_stop_is_idempotent_and_kills_after_failed_ctrl_c(self):
        self.state.touch()
        result = self.run_script("stop", FAKE_CTRL_C_FAIL="1")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(["tmux", "kill-session", "-t", SESSION], self.calls())
        self.assertFalse(self.state.exists())
        self.log.unlink()
        result = self.run_script("stop")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls(), [["tmux", "has-session", "-t", SESSION]])

    def test_stop_reports_a_session_that_cannot_be_killed(self):
        self.state.touch()
        result = self.run_script("stop", FAKE_KILL_FAIL="1")
        self.assertNotEqual(result.returncode, 0)
        self.assertTrue(self.state.exists())

    def test_project_paths_with_spaces_are_shell_safe(self):
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stderr)
        commands = [c[-2] for c in self.calls() if c[0:2] == ["tmux", "send-keys"]
                    and c[-1] == "C-m"]
        for command in commands:
            if command.startswith("cd "):
                resolved = subprocess.run(["bash", "-c", command + " && pwd"],
                                          capture_output=True, text=True, check=True)
                self.assertIn(Path(resolved.stdout.strip()), (self.project, self.project / "src"))
        launch = next(c for c in commands if c.startswith("ros2 launch"))
        decoded = subprocess.run(["bash", "-c", "ros2() { printf '%s\\0' \"$@\"; }; " + launch],
                                 capture_output=True, check=True).stdout.decode().split("\0")[:-1]
        self.assertEqual(decoded, ["launch", "fishbot_navigation2", "navigation_draw_path.launch.py",
                                   "params_file:=" + str(self.config / "nav2_params_height_obstacle.yaml")])
        initial_pose = next(c for c in commands if "initial_pose_tf.py" in c)
        decoded = subprocess.run(["bash", "-c", "python3() { printf '%s\\0' \"$@\"; }; " + initial_pose],
                                 capture_output=True, check=True).stdout.decode().split("\0")[:-1]
        self.assertEqual(decoded, [str(self.project / "scripts/initial_pose_tf.py")])


suite = unittest.defaultTestLoader.loadTestsFromTestCase(DrawPathScriptsTest)
result = unittest.TextTestRunner(verbosity=2).run(suite)
if not result.wasSuccessful():
    sys.exit(1)
print("PASS: drawn path navigation start/stop scripts")
PY
