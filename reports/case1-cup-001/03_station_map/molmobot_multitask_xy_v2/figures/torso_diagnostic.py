"""从本次压缩原始轨迹复现躯干诊断；不改变执行或验收阈值。"""
import base64
import csv
import gzip
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image, ImageOps, ImageDraw
import markdown

ROOT = Path(__file__).resolve().parents[5]
REPORT = Path(__file__).resolve().parents[1]
RUN = ROOT/'cases/case1-cup-001/03_station_map/runs/molmobot_multitask_xy_v2'


def residual(q):
    q = np.asarray(q)
    return float(np.max(np.abs(q-[0., q[1], -2*q[1], q[1], 0., 0.])))


def main():
    records = json.load(open(RUN/'results.json'))
    metrics, paths = [], {}
    for result in records:
        if not result.get('audit_passed'):
            continue
        dest = RUN/'trials'/result['attempt']
        initial = json.load(open(dest/'initial.json'))
        rows = [json.loads(line) for line in gzip.open(dest/'trace.jsonl.gz', 'rt')]
        actions = json.load(open(dest/'actions.json'))
        command_error = max(residual([a['applied_targets'][f'link{i+1}_act'] for i in range(6)]) for a in actions)
        two_fingers = any(set(int(k) for k in r['arms'][s]['finger_normal_forces_N']) == set(r['arms'][s]['expected_fingers']) for r in rows for s in ('left', 'right'))
        metrics.append({'pose_id': result['pose_id'], 'reason': result['reason'], 'duration_s': result['sim_duration_s'],
            'max_command_linkage_error_rad': command_error, 'terminal_actual_linkage_error_rad': residual(rows[-1]['torso']),
            'max_lift_m': max(r['height_m']-initial['height_m'] for r in rows), 'two_force_bearing_fingers_seen': two_fingers})
        paths[result['pose_id']] = rows
    with open(REPORT/'torso_metrics.csv', 'w') as f:
        writer = csv.DictWriter(f, fieldnames=list(metrics[0]))
        writer.writeheader()
        writer.writerows(metrics)
    fig, axs = plt.subplots(1, 2, figsize=(12, 4))
    for key, label, color in [('max_command_linkage_error_rad', 'Commanded linkage residual', '#267eb4'),
                              ('terminal_actual_linkage_error_rad', 'Actual residual at termination', '#c03f40')]:
        axs[0].plot(range(len(metrics)), [m[key] for m in metrics], 'o', ms=4, label=label, color=color)
    axs[0].axhline(.002, color='#555', ls='--', label='Frozen threshold: 0.002 rad')
    axs[0].set(xlabel='Executed start index (A first)', ylabel='Linkage residual (rad)', ylim=(-.0002, .0028))
    axs[0].legend(fontsize=7)
    for pose in ('A', 'P008'):
        rows = paths[pose]
        times = np.array([r['time_s'] for r in rows])
        axs[1].plot(times-times[0]+.004, [residual(r['torso']) for r in rows], label=pose)
    axs[1].axhline(.002, color='#555', ls='--')
    axs[1].set(xlabel='Elapsed simulation time (s)', ylabel='Actual linkage residual (rad)', ylim=(0, .0028))
    axs[1].legend()
    fig.tight_layout()
    fig.savefig(REPORT/'figures/torso_diagnostic.svg')
    fig.savefig(REPORT/'figures/torso_diagnostic.png', dpi=160)
    plt.close(fig)
    # 原始头／腕RGB与实际进入模型的头部鱼眼图并列；不在策略图像中添加标注。
    dest = RUN/'trials/A_attempt_000'
    images = [('Head RGB (raw)', dest/'head_camera_initial.png'), ('Right wrist RGB (raw)', dest/'wrist_camera_r_initial.png'),
              ('Left wrist RGB (raw)', dest/'wrist_camera_l_initial.png'), ('Head after native fisheye', dest/'model_inputs/chunk_000/head_camera.png')]
    canvas = Image.new('RGB', (1024, 624), 'white')
    draw = ImageDraw.Draw(canvas)
    for index, (caption, path) in enumerate(images):
        x, y = index%2*512, index//2*312
        image = ImageOps.contain(Image.open(path).convert('RGB'), (512, 288))
        canvas.paste(image, (x+(512-image.width)//2, y+24+(288-image.height)//2))
        draw.text((x+8, y+5), caption, fill='black')
    canvas.save(REPORT/'figures/visual_inputs.png')
    durations = np.array([m['duration_s'] for m in metrics])
    failures = sum(m['reason']=='torso_linkage' for m in metrics)
    lifts = max(m['max_lift_m'] for m in metrics)
    commands = max(m['max_command_linkage_error_rad'] for m in metrics)
    text = '\n'.join(['## 躯干提前终止诊断', '',
        f'**{failures}/{len(metrics)} 次有效试验因实际躯干联动残差超限提前终止；不是完整40秒抓取尝试后的失败。**', '',
        f'- 命令六关节的联动残差最大值为 **{commands:.6g} rad**，符合 `[0,h,-2h,h,0,0]`。模型并未获得额外躯干自由度。',
        f'- 实测残差在末帧超过冻结的 **0.002 rad**；终止时间范围 **{durations.min():.3f}–{durations.max():.3f} s**，中位数 **{np.median(durations):.3f} s**。',
        f'- 最大目标抬升为 **{lifts:.6f} m**；出现过同一夹爪双指同时真实受力的试验数为 **{sum(m["two_force_bearing_fingers_seen"] for m in metrics)}**。',
        '- 事实：控制目标符合1D联动，实际全身动态响应违反严格联动阈值。推断：当前全身伺服桥／动力学与严格验收之间的不匹配限制了评测。尚未通过干预确认具体是伺服增益、机械耦合、全身反作用或其他因素。',
        '- 因此，本次结论是“当前桥接与冻结协议下没有严格通过”，不能单独据此声称MolmoBot没有视觉抓杯能力。未修改阈值、夹爪或机器人参数补救。', '',
        '![命令与实际躯干联动诊断](figures/torso_diagnostic.png)', '',
        '图：左为每个起点单次试验的命令最大残差与实际终止残差；右为A和P008的4ms原始记录。虚线是固定阈值，无误差棒；不是重复随机试验统计。', '',
        '![A点原始RGB与实际模型头部输入](figures/visual_inputs.png)', '',
        '图：A点头部、右腕和左腕原始RGB，以及上游鱼眼处理后实际进入模型的头部图像；仅在交付拼图上加标题，不修改模型输入。', '',
        '[逐点诊断指标](torso_metrics.csv) · [复现脚本](figures/torso_diagnostic.py)', '',
        '### 后续建议（未执行）', '',
        '先用独立对照测试全身动态动作时的实际躯干保持／联动，再判断控制桥是否需要修复；不要直接放宽本次阈值，或把早停试验解释为40秒模型能力测试。', ''])
    (REPORT/'ANALYSIS.md').write_text(text)
    report = REPORT/'REPORT.md'
    content = report.read_text().split('<!-- torso-diagnostic -->')[0]+'\n<!-- torso-diagnostic -->\n'+text
    warning = f'**注意：35次均为躯干联动约束提前终止（{durations.min():.3f}–{durations.max():.3f}秒），不是完整40秒抓取尝试。不能仅凭本次结果判定模型没有抓杯能力。**'
    anchor = '每个有效起点单次试验；这不是每点成功概率，也不是固定底盘可抓取性。旧 case1 交付和人工门禁未修改。'
    if warning not in content:
        content = content.replace(anchor, anchor+'\n\n'+warning)
    report.write_text(content)
    body = markdown.markdown(content, extensions=['tables', 'toc'])
    for p in (REPORT/'figures').glob('*.png'):
        uri = 'data:image/png;base64,'+base64.b64encode(p.read_bytes()).decode()
        body = body.replace('src="figures/'+p.name+'"', 'src="'+uri+'"')
    style = 'body{max-width:1200px;margin:32px auto;padding:0 24px;font:16px/1.65 sans-serif;color:#25313a}img{max-width:100%;cursor:zoom-in}table{border-collapse:collapse;font-size:13px}td,th{padding:7px;border-bottom:1px solid #ddd;text-align:left}h2{margin-top:36px}a{color:#236ba0}dialog{border:0;padding:0;max-width:95vw;max-height:95vh}dialog img{max-width:95vw;max-height:90vh}dialog::backdrop{background:#000a}@media print{body{margin:0}a{color:inherit}}'
    zoom = '<dialog id="image-zoom"><img></dialog><script>const d=document.getElementById("image-zoom");document.querySelectorAll("body>p img").forEach(i=>i.onclick=()=>{d.querySelector("img").src=i.src;d.showModal()});d.onclick=()=>d.close();</script>'
    (REPORT/'REPORT.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>Case1 MolmoBot VLA</title><style>'+style+'</style>'+body+zoom+'</html>')
    print(json.dumps({'executed_trials': len(metrics), 'torso_linkage_terminations': failures,
        'max_command_residual_rad': commands, 'duration_min_median_max_s': [float(durations.min()), float(np.median(durations)), float(durations.max())]}, indent=2))


if __name__ == '__main__':
    main()
