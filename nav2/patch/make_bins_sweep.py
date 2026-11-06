"""Derive bins_sweep_benchmark.cpp: robot 4 of the footprint sweep, varying mask bin counts."""
import pathlib, sys
d = pathlib.Path(sys.argv[1])
s = (d / "footprint_sweep_benchmark.cpp").read_text()
def sub(old, new):
    global s
    assert s.count(old) == 1, old
    s = s.replace(old, new)
sub("  const bool bitmask = state.range(2) != 0;\n",
    "  const int cfg = static_cast<int>(state.range(2));\n"
    "  const int kBins[] = {0, 64, 128, 128, 128, 256, 512};\n"
    "  const int kSub[] = {0, 2, 1, 2, 4, 2, 2};\n"
    "  const bool bitmask = cfg != 0;\n")
sub('    {"CostCritic.use_footprint_bitmask", bitmask},\n',
    '    {"CostCritic.use_footprint_bitmask", bitmask},\n'
    '    {"CostCritic.footprint_bitmask_heading_bins", bitmask ? kBins[cfg] : 128},\n'
    '    {"CostCritic.footprint_bitmask_subcell_bins", bitmask ? kSub[cfg] : 2},\n')
sub('ArgNames({"robot", "scene", "bitmask"})', 'ArgNames({"robot", "scene", "cfg"})')
sub("->ArgsProduct({{0, 1, 2, 3, 4, 5}, {0, 1}, {0, 1}})", "->ArgsProduct({{1, 4}, {1}, {0, 1, 2, 3, 4, 5, 6}})")
(d / "bins_sweep_benchmark.cpp").write_text(s)
cm = (d / "CMakeLists.txt").read_text()
if "bins_sweep_benchmark" not in cm:
    cm = cm.replace("  fleet_server_benchmark\n", "  fleet_server_benchmark\n  bins_sweep_benchmark\n")
    (d / "CMakeLists.txt").write_text(cm)
print("ok")
