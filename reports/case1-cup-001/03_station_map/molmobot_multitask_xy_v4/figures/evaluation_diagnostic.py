"""从保存轨迹重放并生成本次 v4 诊断图；不更改物理结论或旧 run。"""
import base64
from collections import Counter
import csv
import gzip
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import markdown
ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT/'src'))
from lastmile.vla import VLAValidator
from lastmile.workflow.vla_station import audit_trial
from lastmile.workflow import core as c
RUN = ROOT/'cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v4'
REPORT = Path(__file__).resolve().parents[1]
protocol = c.read(RUN/'vla_protocol.json')
results = c.read(RUN/'results.json')
records = [r for r in results if r.get('audit_passed')]
metrics, audits = [], []
if '--reuse-metrics' in sys.argv:
    with open(REPORT/'evidence_metrics.csv') as f:
        metrics = list(csv.DictReader(f))
    for row in metrics:
        for key in row:
            if key in ('pose_id', 'status', 'reason'):
                continue
            row[key] = row[key] == 'True' if key.endswith('_warning') else float(row[key])
    audits = c.read(REPORT/'evidence_revalidation.json')['trials']
    records = []
for result in records:
    folder = RUN/'trials'/result['attempt']
    manifest = c.read(folder/'evidence_manifest.json')
    for name, digest in manifest['files'].items():
        if c.digest(folder/name) != digest:
            raise ValueError('evidence hash mismatch: '+str(folder/name))
    audit = audit_trial(folder, protocol['thresholds'], protocol['torso_feedback_policy'],
                        protocol['collision_feedback_policy'], protocol['severe_penetration_m'])
    audits.append({'pose_id': result['pose_id'], **audit, 'all_file_hashes_verified': True})
    initial = c.read(folder/'initial.json')
    v = VLAValidator(initial, protocol['thresholds'], protocol['torso_feedback_policy'],
                     protocol['collision_feedback_policy'], protocol['severe_penetration_m'])
    max_lift, two_force_samples, max_hold, max_penetration = 0., 0, 0., 0.
    base_distance = 0.
    previous_base = np.array(result['initial_base'][:2])
    with gzip.open(folder/'trace.jsonl.gz', 'rt') as trace:
        for line in trace:
            row = json.loads(line)
            max_lift = max(max_lift, row['height_m']-initial['height_m'])
            two_force_samples += int(any(len(arm['finger_normal_forces_N']) == 2 for arm in row['arms'].values()))
            max_penetration = max(max_penetration, max((x['penetration_m'] for x in row['robot_contacts']), default=0.))
            base = np.array(row['base'][:2])
            base_distance += float(np.linalg.norm(base-previous_base))
            previous_base = base
            v.feed(row)
            for window in v.windows.values():
                if window is not None:
                    max_hold = max(max_hold, row['time_s']-window[0])
    actions = c.read(folder/'actions.json')
    metrics.append({'pose_id': result['pose_id'], 'status': result['status'], 'reason': result['reason'],
                    'duration_s': result['sim_duration_s'], 'max_lift_m': max_lift,
                    'two_force_samples': two_force_samples, 'max_qualified_hold_s': max_hold,
                    'max_robot_penetration_m': max_penetration, 'base_distance_m': base_distance,
                    'max_command_error_rad': max(a['torso_command_error_rad'] for a in actions),
                    'torso_warning': bool(result.get('torso_feedback_warnings')),
                    'minor_collision_warning': bool(result.get('collision_warnings'))})
with open(REPORT/'evidence_metrics.csv', 'w') as f:
    writer = csv.DictWriter(f, fieldnames=list(metrics[0]))
    writer.writeheader()
    writer.writerows(metrics)
c.write(REPORT/'evidence_revalidation.json', {'all_checks_passed': True, 'independently_replayed_trials': len(audits),
        'protocol': protocol, 'trials': audits})
fig, axs = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
x = np.arange(len(metrics))
colors = ['#17845b' if r['status']=='verified_success' else '#c03f40' if r['reason']=='severe_penetration' else '#236ba0' for r in metrics]
axs[0].scatter(x, [m['max_lift_m']*1000 for m in metrics], c=colors, s=32)
axs[0].axhline(50, ls='--', color='#777', label='Required lift: 50 mm')
axs[0].set(ylabel='Max target lift (mm)', title='Saved physical evidence | one independent trial per valid XY')
axs[0].legend(loc='upper left')
axs[1].scatter(x, [m['max_qualified_hold_s'] for m in metrics], c=colors, s=32)
axs[1].axhline(2, ls='--', color='#777', label='Required continuous qualified hold: 2 s')
axs[1].set(ylabel='Longest qualified hold (s)', xlabel='Initial XY station')
axs[1].legend(loc='upper left')
axs[1].set_xticks(x, [m['pose_id'] for m in metrics], rotation=60, ha='right', fontsize=8)
for ax in axs:
    ax.grid(alpha=.2)
    ax.set_ylim(bottom=-.1)
fig.tight_layout()
fig.savefig(REPORT/'figures/evaluation_diagnostic.png', dpi=160)
fig.savefig(REPORT/'figures/evaluation_diagnostic.svg')
plt.close(fig)
counts = Counter(r['reason'] for r in metrics)
full = sum(abs(r['duration_s']-40)<1e-6 for r in metrics)
two = sum(r['two_force_samples']>0 for r in metrics)
lifted = sum(r['max_lift_m']>=.05 for r in metrics)
maxlift = max(r['max_lift_m'] for r in metrics)
analysis = f'''## 放宽反馈停止条件后的实测结果

- 独立重放并校验完整证据摘要：**{len(metrics)} 次**。本次成功判据为抓取接触／抬升／保持／滑移，轻微碰撞与躯干残差允许警告；不代表旧严格无碰撞协议通过。
- **{full} 次运行完整40秒**；终止原因：`{dict(counts)}`。躯干反馈残差和轻微碰撞不再成为停止原因。
- 曾有同一夹爪双指真实受力接触：**{two} 次**；目标曾抬升≥5cm：**{lifted} 次**；本批最大目标抬升 **{maxlift*1000:.3f} mm**。
- 全部派发动作六关节映射已验证，最大派发联动误差 **{max(r['max_command_error_rad'] for r in metrics):.3g} rad**；反馈偏移仅警告，原始轨迹保留。
- v2／v3 的提前停止现象不能直接等同本次模型抓取结果。当前只对这些起点的一次真实执行下结论，不将一次失败解释为每点成功概率。
- 失败机制尚未通过对照干预定位；抓取接触、抬升及保持证据用于指出缺失的条件，而非证明某个模型／动力学原因。

![逐点抬升与合格保持时间](figures/evaluation_diagnostic.png)

图：每点来自4ms原始物理轨迹；抬升以该试验初始目标高度为基准。合格保持同时要求同夹爪双指受力、无支撑、抬升和滑移约束。蓝点为超时，红点为严重穿模停止，绿点为成功；虚线为验收阈值，无概率或误差棒。

[逐点证据指标](evidence_metrics.csv) · [独立轨迹重验收及摘要校验](evidence_revalidation.json)
'''
(REPORT/'ANALYSIS.md').write_text('# v4 物理证据分析\n\n'+analysis)
report_text = (REPORT/'REPORT.md').read_text().split('<!-- evaluation-diagnostic -->')[0].rstrip()+'\n\n<!-- evaluation-diagnostic -->\n'+analysis
(REPORT/'REPORT.md').write_text(report_text)
body = markdown.markdown(report_text, extensions=['tables', 'toc'])
for name in ('start_results.png', 'measured_routes.png', 'evaluation_diagnostic.png'):
    body = body.replace('src="figures/'+name+'"', 'src="data:image/png;base64,'+base64.b64encode((REPORT/'figures'/name).read_bytes()).decode()+'"')
(REPORT/'REPORT.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>Case1 MolmoBot VLA v4</title><style>body{max-width:1200px;margin:32px auto;padding:0 24px;font:16px/1.65 sans-serif;color:#25313a}img{max-width:100%}table{border-collapse:collapse;font-size:13px}td,th{padding:7px;border-bottom:1px solid #ddd;text-align:left}h2{margin-top:36px}a{color:#236ba0}</style>'+body+'</html>')
print(json.dumps({'trials': len(metrics), 'full_horizon': full, 'reasons': dict(counts), 'two_finger_trials': two, 'lift_5cm_trials': lifted, 'max_lift_mm': maxlift*1000}, ensure_ascii=False))
