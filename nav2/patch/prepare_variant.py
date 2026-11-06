"""Prepare nav2_mppi_controller for one benchmark variant (run inside the container).

  main    upstream main + the CostCritic benchmark of PR #6228's author
  pr      main + PR #6228 (OpenMP, reverse order, early exit, footprint hoisted)
  bitmask pr + footprint bitmask filter + equivalence test

Usage: python3 prepare_variant.py <variant> <nav2 repo> <patch dir>
"""

import pathlib
import subprocess
import sys

variant, repo, patch = sys.argv[1], pathlib.Path(sys.argv[2]), pathlib.Path(sys.argv[3])
pkg = repo / "nav2_mppi_controller"

subprocess.run(["git", "-C", str(repo), "checkout", "--", "."], check=True)
subprocess.run(["git", "-C", str(repo), "clean", "-fdq", "nav2_mppi_controller"], check=True)
if variant in ("pr", "bitmask"):
    subprocess.run(["git", "-C", str(repo), "apply", str(patch / "pr6228.diff")], check=True)
if variant == "bitmask":
    subprocess.run([sys.executable, str(patch / "apply_bitmask.py"), str(pkg)], check=True)

# benchmark of the PR author, with a bitmask switch and whole-process CPU time
bench = (patch / "cost_critic_benchmark.cpp").read_text()
for old, new in [
    ("  const int scene = static_cast<int>(state.range(1));\n",
     "  const int scene = static_cast<int>(state.range(1));\n"
     "  const bool bitmask = state.range(2) != 0;\n"),
    ('    {"CostCritic.num_threads", num_threads},\n',
     '    {"CostCritic.num_threads", num_threads},\n'
     '    {"CostCritic.use_footprint_bitmask", bitmask},\n'),
    ('ArgNames({"threads", "scene"})', 'ArgNames({"threads", "scene", "bitmask"})'),
    ("->ArgsProduct({{1, 2, 4, 8}, {0, 1}})", "->ArgsProduct({{1, 2, 4, 8}, {0, 1}, {0, 1}})"),
    ("->UseRealTime()", "->UseRealTime()->MeasureProcessCPUTime()"),
    # CriticData changed on main since the benchmark was written
    ("  {robot_state, trajectories, path, goal, costs, std::nullopt, model_dt,\n"
     "    false, nullptr, nullptr, std::nullopt, std::nullopt};",
     "  {robot_state, trajectories, path, goal, costs, model_dt,\n"
     "    false, nullptr, nullptr, std::nullopt, std::nullopt, {}};"),
]:
    assert bench.count(old) == 1, old
    bench = bench.replace(old, new)
(pkg / "benchmark" / "cost_critic_benchmark.cpp").write_text(bench)

names = ["cost_critic_benchmark"] + (["cost_critic_equivalence"] if variant == "bitmask" else [])
if variant == "bitmask":
    (pkg / "benchmark" / "cost_critic_equivalence.cpp").write_text(
        (patch / "cost_critic_equivalence.cpp").read_text())
cm = (patch / "bench_CMakeLists.txt").read_text()
cm = cm.replace("  optimizer_benchmark\n  controller_benchmark\n  cost_critic_benchmark\n",
                "".join(f"  {n}\n" for n in names))
(pkg / "benchmark" / "CMakeLists.txt").write_text(cm)

# build only the benchmarks, not the unit tests
top = (pkg / "CMakeLists.txt").read_text()
assert top.count("  add_subdirectory(test)\n") == 1
(pkg / "CMakeLists.txt").write_text(top.replace("  add_subdirectory(test)\n", ""))
print("prepared", variant)
