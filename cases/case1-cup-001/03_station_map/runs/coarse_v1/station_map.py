#!/usr/bin/env python3
# 原始证据上的可复现站位图入口；绘图实现见 src/lastmile/workflow/station_plot.py
from pathlib import Path
import subprocess
p=Path(__file__).resolve();W=p.parents[5]
subprocess.check_call([str(W/'bin/lastmile'),'plot-station-map','--case-id',p.parents[3].name,'--run-id',p.parent.name])
