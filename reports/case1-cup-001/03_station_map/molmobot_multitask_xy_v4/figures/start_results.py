from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[5]/'src'))
from lastmile.workflow.vla_station_report import export_vla_report
export_vla_report('case1-cup-001','coarse_v1','molmobot_multitask_xy_v4')
