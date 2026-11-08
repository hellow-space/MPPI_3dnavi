#!/usr/bin/env python3
"""Run the dry-run's actual terminal command with a fake Docker executable."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ElevationTerminalExitTest(unittest.TestCase):
    def test_elevation_terminal_exits_with_docker_in_both_profiles(self):
        for profile in ("height_jump", "height_obstacle"):
            preview = subprocess.run(
                ["/bin/bash", str(ROOT / "scripts" / f"start_all_{profile}.sh"), "--dry-run"],
                check=True, capture_output=True, text=True)
            terminal_line = next(
                line.removeprefix("[预览] ")
                for line in preview.stdout.splitlines()
                if line.startswith("[预览] gnome-terminal "))
            # Decode Bash printf %q (including $'...' multiline arguments)
            # through a shell function that only records arguments.
            captured = subprocess.run(
                ["/bin/bash", "-c", "gnome-terminal() { printf '%s\\0' \"$@\"; }; "
                 + terminal_line], check=True, capture_output=True)
            terminal_args = captured.stdout.decode().rstrip("\0").split("\0")
            command = terminal_args[terminal_args.index("-c") + 1]
            with tempfile.TemporaryDirectory() as tmp:
                fake_bin = Path(tmp)
                docker = fake_bin / "docker"
                docker.write_text('#!/bin/sh\nexit "$FAKE_DOCKER_EXIT"\n')
                docker.chmod(0o755)
                retained_shell = fake_bin / "bash"
                retained_shell.write_text('#!/bin/sh\nexit 77\n')
                retained_shell.chmod(0o755)
                for docker_exit in (0, 137):
                    with self.subTest(profile=profile, docker_exit=docker_exit):
                        env = dict(os.environ, PATH=tmp + os.pathsep + os.environ["PATH"],
                                   FAKE_DOCKER_EXIT=str(docker_exit))
                        result = subprocess.run(["/bin/bash", "-c", command], env=env,
                                                capture_output=True, text=True, timeout=3)
                        self.assertEqual(
                            result.returncode, docker_exit,
                            "terminal command must exit with Docker, rather than open a shell")


if __name__ == "__main__":
    unittest.main()
