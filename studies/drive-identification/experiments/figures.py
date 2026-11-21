"""Rebuild the figures and verify the complete paired design from stored data."""
import itertools
import json
from pathlib import Path
from statistics import median

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from mpp.world.layout import warehouse

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "results"
FIG = ROOT / "figures"
FIG.mkdir(exist_ok=True)
rows = [json.loads(line) for line in (DATA / "ablation.jsonl").read_text().splitlines()]
design = {(r["seed"], r["planner"], r["onboard"]) for r in rows}
assert len(rows) == 64 and design == set(itertools.product(range(301,317), (False,True), (False,True)))
plt.rcParams.update({"font.size":12, "axes.spines.top":False,"axes.spines.right":False,
                     "savefig.bbox":"tight","figure.constrained_layout.use":True})
def save(fig, name):
    for ext in ("png","pdf"):
        fig.savefig(FIG / f"{name}.{ext}",dpi=180)
    plt.close(fig)

fig,axes = plt.subplots(2,1,figsize=(7.2,6.8))
for p, ax, title in zip((0,1),axes,("Кинематическая модель","ARX в планировщике")):
    data = np.array(json.loads((DATA/f"route-303-{p}0.json").read_text()))
    for obj in warehouse().rects:
        ax.add_patch(Rectangle((obj.x0,obj.y0),obj.x1-obj.x0,obj.y1-obj.y0,color="#c8ccd0"))
    ax.add_patch(Rectangle((4.82,3.245),.36,.36,facecolor="#292929",hatch="xxx"))
    ax.plot(data[:,1],data[:,2],color="#172b40",lw=2)
    ax.scatter(data[0,1],data[0,2],s=35,facecolor="white",edgecolor="black",zorder=4)
    ax.scatter(data[-1,1],data[-1,2],s=44,marker="x",color="black")
    ax.scatter(9.4,3.425,marker="*",s=90,color="#777777")
    ax.set(xlim=(0,10),ylim=(0,7),aspect="equal",title=title,ylabel="y, м",xlabel="x, м")
save(fig,"routes-303")

groups=[(0,0,"Кинематика"),(0,1,"ARX на борту"),(1,0,"ARX в MPPI"),(1,1,"ARX в обоих блоках")]
fig,ax=plt.subplots(figsize=(7.3,3.0))
for j,(p,b,label) in enumerate(groups):
    group=[r for r in rows if (r["planner"],r["onboard"])==(p,b)]
    count=sum(r["success"] for r in group)
    ax.barh(j,count,color=["#9ba0a4","#757c80","#37424a","#1b272f"][j],height=.55)
    ax.text(count+.18,j,f"{count}/16",va="center")
ax.set(yticks=range(4),yticklabels=[g[2] for g in groups],xticks=[0,4,8,12,16],xlim=(0,18.8),xlabel="Успешные заезды")
ax.invert_yaxis()
save(fig,"placement")

records=(DATA/"nav2-summary.txt").read_text().splitlines()
values={}
for key,line in zip(records[::2],records[1::2],strict=True):
    run,wall,build=key.split()
    values[int(run),int(wall),build]=float(dict(t.split("=") for t in line.split())["robots@20Hz"])
assert len(values)==12
fig,ax=plt.subplots(figsize=(7.3,3.4))
for offset,b,label,c in [(-.16,"ws_main","Базовая сборка","#9ba0a4"),(.16,"ws","Опытная сборка","#37424a")]:
    ys=[median(values[r,w,b] for r in (1,2,3)) for w in (0,1)]
    bars=ax.bar(np.arange(2)+offset,ys,width=.29,label=label,color=c)
    ax.bar_label(bars,labels=[str(v).replace(".",",") for v in ys],padding=3)
ax.set(xticks=[0,1],xticklabels=["Коридор","У стены"],ylabel="Эквиваленты при 20 Гц",ylim=(0,110))
ax.legend(ncols=2,loc="upper center",frameon=False)
save(fig,"nav2-throughput")
