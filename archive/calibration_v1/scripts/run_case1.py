"""真实导航 -> 保留实测到站状态 -> cuRobo 重新规划 -> 严格 Pick。"""
import argparse,copy,json,traceback
from pathlib import Path
import numpy as np
import mujoco
from lastmile.simulation import Simulation,body_pose
from lastmile.navigation import execute_navigation
from lastmile.calibration_runner import run_one
from lastmile.audit import write_json,sha256
p=argparse.ArgumentParser();p.add_argument('--run-id',required=True);p.add_argument('--trials',type=int,default=1);a=p.parse_args()
W=Path(__file__).absolute().parents[1];config=json.loads((W/'configs/case1_r_001.json').read_text());root=W/'runs/case1'/a.run_id
if root.exists():raise ValueError('不覆盖历史证据')
sim=Simulation.from_cached(W,14,W/'runs/stage_a/E002-corrected/0014_seed0',root)
# 只在派生 case 初始配置中移动机器人；执行中严禁直接写底盘 qpos。
for name,q in zip(['base_x','base_y','base_theta'],config['start_base']):sim.data.qpos[sim.qpos_adr[name]]=q
mujoco.mj_forward(sim.model,sim.data);sim.hold_ctrl();mujoco.mj_getState(sim.model,sim.data,sim.initial_state,sim.state_spec)
np.savez_compressed(root/'case_initial_snapshot.npz',state=sim.initial_state)
write_json(root/'case_config.json',config)
source_episode=copy.deepcopy(sim.episode);results=[]
for trial in range(a.trials):
    dest=root/f'trial_{trial:02d}';dest.mkdir();sim.episode=copy.deepcopy(source_episode);sim.restore();sim.tick_observer=None
    nav_trace=[];states=[{'time_s':float(sim.data.time),'phase':'start','qpos':sim.data.qpos.copy()}]
    try:
        arrival=execute_navigation(sim,np.array(config['nav_path']),np.array(config['goal_base']),nav_trace,states,max_speed=config['nav_max_speed_m_s'])
        write_json(dest/'navigation_trace.json',{'initial_base':config['start_base'],'arrival':arrival,'samples':nav_trace})
        state=np.zeros_like(sim.initial_state);mujoco.mj_getState(sim.model,sim.data,state,sim.state_spec);np.savez_compressed(dest/'actual_arrival_snapshot.npz',state=state)
        # 原校准器比较的 base 参考必须是实际到站值，不能把到站误差当抓取漂移。
        sim.episode['robot']['init_qpos']['base']=sim.group('base').tolist()
        begin_base=sim.group('base').copy();begin_time=float(sim.data.time);counter=[0]
        def observer(s,phase):
            counter[0]+=1
            if counter[0]%10==0:states.append({'time_s':float(s.data.time),'phase':phase,'qpos':s.data.qpos.copy()})
        sim.tick_observer=observer
        pick=run_one(W,14,'left',.738,trial,dest/'pick',1,sim=sim,world_geometry='mesh',time_dilation=.25,approach_overshoot=.01,grasp_ids=[794],restore_start=False)
        result={'trial_id':f'{a.run_id}-{trial:02d}','trial':trial,'status':pick['status'],'reason':pick['reason'],'arrival':arrival,'pick':pick,'arrival_used_without_reset':True,'navigation_end_time_s':begin_time,'pick_start_base':begin_base.tolist(),'total_physical_time_s':float(sim.data.time),'case_config_sha256':sha256(W/'configs/case1_r_001.json'),'protocol_sha256':sha256(W/'configs/protocol_v1.json')}
    except Exception as exc:
        traceback.print_exc();result={'trial_id':f'{a.run_id}-{trial:02d}','trial':trial,'status':'executed_failure','reason':str(exc),'partial_navigation_samples':len(nav_trace)}
        write_json(dest/'navigation_trace.json',{'samples':nav_trace,'error':str(exc)})
    finally:
        sim.tick_observer=None
        np.savez_compressed(dest/'video_states.npz',qpos=np.array([r['qpos'] for r in states],dtype=np.float32),time_s=np.array([r['time_s'] for r in states]),phase=np.array([r['phase'] for r in states]))
        write_json(dest/'result.json',result);results.append(result);write_json(W/f'reports/{a.run_id}_case1.json',{'case_id':config['case_id'],'trials':results,'successes':sum(r['status']=='verified_success' for r in results),'expected_trials':a.trials})
        print(json.dumps(result,ensure_ascii=False),flush=True)
    if result['status']!='verified_success':break
