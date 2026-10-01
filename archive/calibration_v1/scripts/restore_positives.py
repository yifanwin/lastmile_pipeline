#!/usr/bin/env python3
import argparse
from pathlib import Path
from lastmile.simulation import Simulation
p=argparse.ArgumentParser();p.add_argument('--index',type=int,required=True);a=p.parse_args()
W=Path(__file__).absolute().parents[1]
s=Simulation(W,a.index,W/f'runs/restoration/{a.index:04d}')
print('restored',a.index,s.sample('left',0),flush=True)
