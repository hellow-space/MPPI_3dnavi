#!/usr/bin/env python3
"""Run existing tests without starting navigation, simulation, or hardware."""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros-tests", action="store_true",
                        help="also run prebuilt map and plugin tests (source ROS/workspace first)")
    parser.add_argument("--benchmark", action="store_true",
                        help="measure synthetic evaluator only, excluding GridMap and controller")
    parser.add_argument("--terrain-tests", action="store_true",
                        help="fresh standalone build of real GridMap query/reference tests (requires ROS libraries)")
    parser.add_argument("--terrain-benchmark", action="store_true",
                        help="also time real GridMap conversion and 1000x56 scoring; implies --terrain-tests")
    parser.add_argument("--height-tests", action="store_true",
                        help="fresh standalone build of real height layer/costmap integration tests")
    parser.add_argument("--height-benchmark", action="store_true",
                        help="also compare legacy/optimized whole-map scan kernels; implies --height-tests")
    parser.add_argument("--output", type=Path, required=True,
                        help="new report directory; existing directories are rejected")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True, exist_ok=False)
    output = args.output.resolve()
    report = {"created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "scope": "Test wall_seconds include process startup; synthetic benchmark percentiles are in its log",
              "workspace": str(root), "platform": platform.platform(),
              "options": vars(args).copy(), "results": [], "source_sha256": {}}
    report["options"]["output"] = str(output)
    # Include untracked test files too; git diff alone cannot reproduce their contents.
    for directory in ("src/height_jump_mppi_critic", "src/height_obstacle_layer",
                      "tests/offline_terrain", "tests/offline_height_layer"):
        for path in sorted((root / directory).rglob("*")):
            if path.is_file() and path.suffix in (".cpp", ".hpp", ".txt"):
                report["source_sha256"][str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    for path in (Path(__file__).resolve(), root / "tests/benchmark_height_evaluator.cpp"):
        report["source_sha256"][str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()

    def run(name, command, timeout=120):
        start = time.monotonic()
        try:
            result = subprocess.run(command, cwd=root, text=True, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, timeout=timeout, check=False)
            code, log = result.returncode, result.stdout
        except (OSError, subprocess.TimeoutExpired) as error:
            code, log = 1, str(error)
            if isinstance(error, subprocess.TimeoutExpired) and error.stdout:
                log += "\n" + (error.stdout.decode(errors="replace") if isinstance(error.stdout, bytes)
                               else error.stdout)
        (output / (name + ".log")).write_text(log, encoding="utf-8")
        report["results"].append({"name": name, "command": command, "returncode": code,
                                  "wall_seconds": time.monotonic() - start})
        print(f"{'PASS' if code == 0 else 'FAIL'} {name}", flush=True)
        return code == 0

    run("git_head", ["git", "rev-parse", "HEAD"])
    run("git_status", ["git", "status", "--short"])
    run("git_diff", ["git", "diff", "HEAD"])
    run("compiler_version", [os.environ.get("CXX", "c++"), "--version"])
    for package in ("height_jump_mppi_critic", "height_obstacle_layer"):
        cache = root / "build" / package / "CMakeCache.txt"
        if cache.exists():
            (output / (package + "_CMakeCache.txt")).write_bytes(cache.read_bytes())
    # Compile only standalone tests into a temporary directory. Do not alter install/build.
    with tempfile.TemporaryDirectory(prefix="height-offline-") as temporary:
        for name in ("test_height_obstacle_classifier", "test_height_obstacle_snapshot"):
            binary = str(Path(temporary) / name)
            if run(name + "_compile", [os.environ.get("CXX", "c++"), "-std=c++17", "-O2",
                                      "-fno-fast-math", "-UNDEBUG", "-I",
                                      str(root / "src/height_obstacle_layer/include"),
                                      str(root / "tests" / (name + ".cpp")), "-o", binary]):
                run(name, [binary])
        if args.benchmark:
            binary = str(Path(temporary) / "benchmark_height_evaluator")
            if run("benchmark_compile", [os.environ.get("CXX", "c++"), "-std=c++17",
                                         "-O3", "-fno-fast-math", "-I",
                                         str(root / "src/height_jump_mppi_critic/include"),
                                         str(root / "tests/benchmark_height_evaluator.cpp"),
                                         "-o", binary]):
                run("benchmark_height_evaluator", [binary])
        if args.terrain_tests or args.terrain_benchmark:
            build = str(Path(temporary) / "terrain_build")
            configured = run("terrain_configure", ["cmake", "-S", str(root / "tests/offline_terrain"),
                                                   "-B", build, "-DCMAKE_BUILD_TYPE=Release"])
            if configured and run("terrain_build", ["cmake", "--build", build, "--parallel", "2"], timeout=300):
                binary = str(Path(build) / "validate_terrain")
                report["terrain_binary_sha256"] = hashlib.sha256(Path(binary).read_bytes()).hexdigest()
                for relative in ("CMakeCache.txt", "CMakeFiles/validate_terrain.dir/flags.make"):
                    path = Path(build) / relative
                    if path.exists():
                        (output / ("terrain_" + path.name)).write_bytes(path.read_bytes())
                run("terrain_correctness", [binary])
                run("terrain_map_regression", [str(Path(build) / "test_terrain_map")])
                # Benchmark uses interior points only and is independent of edge audit failures.
                # The overall report still fails if any correctness check failed.
                if args.terrain_benchmark:
                    run("terrain_benchmark", [binary, "--benchmark"], timeout=300)
        if args.height_tests or args.height_benchmark:
            build = str(Path(temporary) / "height_build")
            configured = run("height_configure", ["cmake", "-S", str(root / "tests/offline_height_layer"),
                                                  "-B", build, "-DCMAKE_BUILD_TYPE=Release"])
            if configured and run("height_build", ["cmake", "--build", build, "--parallel", "2"], timeout=300):
                report["height_binary_sha256"] = {}
                for name in ("test_height_layer", "test_sampling", "benchmark_scan"):
                    report["height_binary_sha256"][name] = hashlib.sha256((Path(build) / name).read_bytes()).hexdigest()
                for relative in ("CMakeCache.txt", "CMakeFiles/test_height_layer.dir/flags.make"):
                    path = Path(build) / relative
                    if path.exists():
                        (output / ("height_" + path.name)).write_bytes(path.read_bytes())
                run("height_layer_integration", [str(Path(build) / "test_height_layer")])
                run("height_sampling", [str(Path(build) / "test_sampling")])
                if args.height_benchmark:
                    run("height_scan_benchmark", [str(Path(build) / "benchmark_scan")], timeout=300)
    if args.ros_tests:
        for package, names in (
            ("height_jump_mppi_critic", ("test_evaluator", "test_map", "test_plugin")),
            ("height_obstacle_layer", ("test_height_difference", "test_snapshot_policy", "test_sampling", "test_layer")),
        ):
            for name in names:
                run(package + "_" + name, [str(root / "build" / package / name)])
    report["passed"] = all(item["returncode"] == 0 for item in report["results"])
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Report: {output / 'report.json'}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
