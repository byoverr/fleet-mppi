"""Add the footprint bitmask filter to CostCritic (on top of the PR #6228 version).

Usage: python3 apply_bitmask.py <path to nav2_mppi_controller>
"""

import pathlib
import shutil
import sys

pkg = pathlib.Path(sys.argv[1])
hpp = pkg / "include/nav2_mppi_controller/critics/cost_critic.hpp"
cpp = pkg / "src/critics/cost_critic.cpp"
shutil.copy(
    pathlib.Path(__file__).with_name("footprint_bitmask.hpp"),
    pkg / "include/nav2_mppi_controller/tools/footprint_bitmask.hpp",
)


def sub(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)


h = hpp.read_text()
h = sub(
    h,
    '#include "nav2_mppi_controller/tools/utils.hpp"\n',
    '#include "nav2_mppi_controller/tools/utils.hpp"\n'
    '#include "nav2_mppi_controller/tools/footprint_bitmask.hpp"\n',
)
h = sub(
    h,
    """    {
      score_cost = static_cast<float>(collision_checker_.footprintCostAtPose(""",
    """    {
      // the bitmask certifies most poses free without the exact (perimeter) footprint check
      if (use_bitmask_ && fp_bitmask_.ready() &&
        !fp_bitmask_.mayCollide(
          static_cast<double>(x), static_cast<double>(y), static_cast<double>(theta)))
      {
        return false;
      }
      score_cost = static_cast<float>(collision_checker_.footprintCostAtPose(""",
)
h = sub(
    h,
    "  int num_threads_{-1};\n",
    "  int num_threads_{-1};\n"
    "  bool use_bitmask_{true};\n"
    "  int bitmask_heading_bins_{128};\n"
    "  int bitmask_subcell_bins_{2};\n"
    "  mppi::FootprintBitmask fp_bitmask_;\n",
)
hpp.write_text(h)

c = cpp.read_text()
c = sub(
    c,
    '  getParam(num_threads_, "num_threads", -1);\n',
    '  getParam(num_threads_, "num_threads", -1);\n'
    '  getParam(use_bitmask_, "use_footprint_bitmask", true);\n'
    '  getParam(bitmask_heading_bins_, "footprint_bitmask_heading_bins", 128);\n'
    '  getParam(bitmask_subcell_bins_, "footprint_bitmask_subcell_bins", 2);\n',
)
c = sub(
    c,
    "    footprint = costmap_ros_->getRobotFootprint();\n",
    "    footprint = costmap_ros_->getRobotFootprint();\n"
    "    if (use_bitmask_) {\n"
    "      fp_bitmask_.configure(\n"
    "        footprint, costmap->getResolution(), bitmask_heading_bins_, bitmask_subcell_bins_);\n"
    "      fp_bitmask_.buildPlane(*costmap, is_tracking_unknown_);\n"
    "    }\n",
)
cpp.write_text(c)
print("patched", hpp, cpp)
