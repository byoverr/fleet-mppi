"""Download the selected papers into the Desktop folder, politely (arXiv asks for ~3 s between requests)."""
import os
import shutil
import time
import urllib.request

OUT = os.path.expanduser("~/Desktop/Статьи про эффективность планировщиков")
OLD = os.path.expanduser("~/Desktop/Статьи для практики")
UA = "Mozilla/5.0 (literature download for personal study)"
os.makedirs(OUT, exist_ok=True)

ARXIV = [
    ("02_CAPT_Ramsey_2024", "2406.02807"),
    ("03_FCIT-star_Wilson_2025", "2411.17902"),
    ("04_Vec-QMDP_Jin_2026", "2602.08334"),
    ("05_Think_Fast_and_Far_Liang_2026", "2606.04355"),
    ("06_Multi-Robot_Motions_in_Milliseconds_Motes_2026", "2604.23960"),
    ("07_VAMP-MR_Huang_2026", "2607.13478"),
    ("09_SE2_Navigation_Mesh_Shi_2026", "2607.01454"),
    ("10_Rectangular_Corridor_Gonzalez-Garcia_2026", "2602.09714"),
    ("11_Check_Ordering_Jiang_2026", "2609.30599"),
    ("12_Slender_Links_EDF_Ecker_2025", "2507.01705"),
    ("13_MPPI-Generic_Vlahov_2024", "2409.07563"),
    ("14_MPPI_Hardware_Tanguy-Legac_2026", "2601.12089"),
    ("15_MPPI_FPGA_Desai_2026", "2601.17231"),
    ("16_LP-MPPI_Kicki_2026", "2503.11717"),
    ("17_Feedback-MPPI_Belvedere_2026", "2506.14855"),
    ("19_GATO_Du_2026", "2510.07625"),
    ("20_Edge_Quality_of_Control_Roy_2026", "2607.14853"),
    ("21_Action_Chunk_Scheduling_Bansal_2026", "2608.00337"),
    ("22_FogROS2-FT_Chen_2024", "2412.05408"),
    ("23_Speculative_Policy_Orchestration_Nguyen_2026", "2603.19418"),
]
OTHER = [
    ("08_RACOD_Bakhshalipour_2022", "https://par.nsf.gov/servlets/purl/10414984"),
    ("24_Agents_of_Autonomy_Bakhshalipour_2023", "https://dl.acm.org/doi/pdf/10.1145/3626774"),
    ("25_Non-Circular_Footprint_Convolution_Sensors_2017", "https://www.mdpi.com/1424-8220/17/8/1876/pdf"),
    ("26_Vectorization_Intersection_Kasmynin_2026", "https://link.springer.com/content/pdf/10.1007/s11370-025-00680-4.pdf"),
]
COPIES = [
    ("01_VAMP_Thomason_2024", "22_VAMP_Thomason_2024.pdf"),
    ("18_Biased-MPPI_Trevisan_2024", "14_Biased-MPPI_Trevisan_2024.pdf"),
]


def get(name, url):
    path = os.path.join(OUT, name + ".pdf")
    if os.path.exists(path):
        print("exists", name)
        return
    try:
        req = urllib.request.Request(url, headers={"User-Agent": UA})
        with urllib.request.urlopen(req, timeout=120) as r:
            data = r.read()
        if not data.startswith(b"%PDF"):
            print(f"NOT A PDF ({len(data)} bytes)", name, url)
            return
        with open(path, "wb") as f:
            f.write(data)
        print(f"ok {len(data) / 1e6:.1f} MB", name)
    except Exception as e:  # a paywall or a block is reported, not worked around
        print("FAILED", name, url, repr(e))


for name, src in COPIES:
    shutil.copy2(os.path.join(OLD, src), os.path.join(OUT, name + ".pdf"))
    print("copied", name)
for name, aid in ARXIV:
    get(name, f"https://arxiv.org/pdf/{aid}")
    time.sleep(3.2)
for name, url in OTHER:
    get(name, url)
    time.sleep(1.0)
