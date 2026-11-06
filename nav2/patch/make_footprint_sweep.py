"""Derive footprint_sweep_benchmark.cpp from the prepared cost_critic_benchmark.cpp.

Same scenes and trajectories, but footprint, map resolution, inflation radius and wall distance
vary per benchmark argument (robot index). One thread; bitmask off/on.
Usage: python3 make_footprint_sweep.py <benchmark dir>
"""

import pathlib
import sys

d = pathlib.Path(sys.argv[1])
s = (d / "cost_critic_benchmark.cpp").read_text()


def sub(old, new):
    global s
    assert s.count(old) == 1, old
    s = s.replace(old, new)


sub("constexpr double kResolution = 0.1;", "double kResolution = 0.1;")
sub("constexpr double kWallDistance = 1.7;",
    "double kWallDistance = 1.7;\n"
    "std::string kFootprint;\n"
    "double kInflation = 5.0;\n"
    "struct Robot {const char * name; const char * footprint; double res, inflation, wall;};\n"
    "const Robot kRobots[] = {\n"
    "  {\"turtlebot_0.2x0.18@0.05\", \"[[0.1,0.09],[-0.1,0.09],[-0.1,-0.09],[0.1,-0.09]]\", 0.05, 0.6, 0.45},\n"
    "  {\"amr_0.7x0.5@0.05\", \"[[0.35,0.25],[-0.35,0.25],[-0.35,-0.25],[0.35,-0.25]]\", 0.05, 1.5, 0.6},\n"
    "  {\"amr_0.7x0.5@0.025\", \"[[0.35,0.25],[-0.35,0.25],[-0.35,-0.25],[0.35,-0.25]]\", 0.025, 1.5, 0.6},\n"
    "  {\"forklift_2.4x1.2@0.05\", \"[[1.6,0.6],[-0.8,0.6],[-0.8,-0.6],[1.6,-0.6]]\", 0.05, 3.0, 0.95},\n"
    "  {\"big_3.66x1.3@0.1\", \"[[2.230,0.65],[-1.4312,0.65],[-1.4312,-0.65],[2.230,-0.65]]\", 0.1, 5.0, 1.7},\n"
    "  {\"big_3.66x1.3@0.05_no_masks\", \"[[2.230,0.65],[-1.4312,0.65],[-1.4312,-0.65],[2.230,-0.65]]\", 0.05, 5.0, 1.7},\n"
    "};")
sub('    {"footprint", "[ [2.230, 0.65], [-1.4312, 0.65], [-1.4312, -0.65], [2.230, -0.65] ]"},\n',
    '    {"footprint", kFootprint},\n')
sub('    {"inflation_layer.inflation_radius", 5.0},\n', '    {"inflation_layer.inflation_radius", kInflation},\n')
sub("  const int num_threads = static_cast<int>(state.range(0));\n",
    "  const Robot & robot = kRobots[state.range(0)];\n"
    "  kFootprint = robot.footprint; kResolution = robot.res; kInflation = robot.inflation;\n"
    "  kWallDistance = robot.wall;\n"
    "  state.SetLabel(robot.name);\n"
    "  const int num_threads = 1;\n")
sub('ArgNames({"threads", "scene", "bitmask"})', 'ArgNames({"robot", "scene", "bitmask"})')
sub("->ArgsProduct({{1, 2, 4, 8}, {0, 1}, {0, 1}})", "->ArgsProduct({{0, 1, 2, 3, 4, 5}, {0, 1}, {0, 1}})")
(d / "footprint_sweep_benchmark.cpp").write_text(s)
cm = (d / "CMakeLists.txt").read_text()
if "footprint_sweep_benchmark" not in cm:
    cm = cm.replace("  full_cycle_benchmark\n", "  full_cycle_benchmark\n  footprint_sweep_benchmark\n")
    (d / "CMakeLists.txt").write_text(cm)
print("ok")
