"""从逐点证据生成 Markdown、自包含 HTML 和起点／实测路线图。"""
import base64
from collections import Counter
import csv
import html
import json
import os
from pathlib import Path
import numpy as np
from . import core as c

COLORS = {'verified_success': '#17845b', 'executed_failure': '#c03f40', 'invalid_initialization': '#777777',
          'infrastructure_error': '#7953a0', 'not_run': '#c28c20'}
LABELS = {'verified_success': 'Success', 'executed_failure': 'Failure', 'invalid_initialization': 'Invalid start',
          'infrastructure_error': 'Infrastructure error', 'not_run': 'Not run'}


def export_vla_report(case_id, station_run_id, run_id):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import markdown
    folder = c.case_dir(case_id)/c.STAGES[2]/'runs'/run_id
    if not (folder/'selected_starts.json').exists():
        return
    report = c.WORKSPACE/'reports'/case_id/c.STAGES[2]/run_id
    figures = report/'figures'
    figures.mkdir(parents=True, exist_ok=True)
    starts = c.read(folder/'selected_starts.json')
    records = c.read(folder/'results.json') if (folder/'results.json').exists() else []
    by_id = {r['pose_id']: r for r in records}
    rows = [by_id.get(s['pose_id'], {'pose_id': s['pose_id'], 'initial_base': s['base'], 'status': 'not_run', 'reason': 'not_run'}) for s in starts]
    protocol = c.read(folder/'vla_protocol.json') if (folder/'vla_protocol.json').exists() else {}
    feedback_policy = protocol.get('torso_feedback_policy', 'strict')
    collision_policy = protocol.get('collision_feedback_policy', 'strict')
    collision_rows = [r for r in rows if r.get('collision_warnings')]
    collision_warning_samples = sum(w['samples'] for r in collision_rows for w in r['collision_warnings'].values())
    max_penetration = max((w['max_penetration_m'] for r in collision_rows for w in r['collision_warnings'].values()), default=0.)
    warning_rows = [r for r in rows if r.get('torso_feedback_warnings')]
    warning_samples = sum(w['samples'] for r in warning_rows for w in r['torso_feedback_warnings'].values())
    max_residual = max((w['max_residual_rad'] for r in warning_rows for w in r['torso_feedback_warnings'].values()), default=0.)
    counts = Counter(r['status'] for r in rows)
    state = c.read(folder/'run_state.json') if (folder/'run_state.json').exists() else {'status': 'running'}
    summary = {'case_id': case_id, 'run_id': run_id, 'status': state['status'], 'xy_records': len(starts), 'counts': dict(counts),
               'executed_trials': counts['verified_success']+counts['executed_failure'], 'per_point_repeats': 1,
               'semantics': 'visual_mobile_pick_from_start_not_fixed_base_station_graspability',
               'outcome_proportion_is_not_per_point_probability': True, 'torso_feedback_policy': feedback_policy,
               'torso_warning_trials': len(warning_rows), 'torso_warning_category_samples': warning_samples,
               'max_torso_warning_residual_rad': max_residual, 'collision_feedback_policy': collision_policy,
               'minor_collision_warning_trials': len(collision_rows), 'collision_warning_category_samples': collision_warning_samples,
               'max_minor_collision_penetration_m': max_penetration, 'updated_at_utc': c.now()}
    c.write(folder/'summary.json', summary)
    facts = c.read(c.case_dir(case_id)/c.STAGES[0]/'scene_facts.json')
    center, size = np.array(facts['support_AABB_center']), np.array(facts['support_AABB_size'])
    target_pose = np.asarray(facts['target_initial_pose'])
    target = target_pose[:3] if target_pose.shape == (7,) else target_pose[:3, 3]
    projection = c.read(c.case_dir(case_id)/c.STAGES[2]/'runs'/station_run_id/'scene_projection.json')
    xy = np.array([s['base'][:2] for s in starts])
    low, high = xy.min(0)-.5, xy.max(0)+.5
    def background(ax):
        from matplotlib.patches import Rectangle
        for item in projection:
            lo, hi = np.array(item['min']), np.array(item['max'])
            if np.all(hi[:2] >= low) and np.all(lo[:2] <= high):
                ax.add_patch(Rectangle(lo[:2], *(hi[:2]-lo[:2]), color='#dedede', alpha=.35))
        ax.add_patch(Rectangle(center[:2]-size[:2]/2, *size[:2], facecolor='#ead5b1', edgecolor='#786446'))
        ax.scatter(*target[:2], marker='*', s=100, color='#e49a23', zorder=5)
        ax.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), xlabel='World x (m)', ylabel='World y (m)')
        ax.set_aspect('equal')
        ax.grid(alpha=.15)
    fig, ax = plt.subplots(figsize=(11, 9))
    background(ax)
    for status in COLORS:
        subset = [r for r in rows if r['status'] == status]
        if subset:
            points = np.array([r['initial_base'][:2] for r in subset])
            ax.scatter(points[:, 0], points[:, 1], color=COLORS[status], marker='x' if status == 'invalid_initialization' else 'o',
                       s=65, label=f'{LABELS[status]} (n={len(subset)})', zorder=6)
    for r in rows:
        ax.annotate(r['pose_id'], r['initial_base'][:2], xytext=(4, 4), textcoords='offset points', fontsize=7)
        if r.get('attempt') and (folder/'trials'/r['attempt']/'states.npz').exists():
            z = np.load(folder/'trials'/r['attempt']/'states.npz')
            # base_x/y are resolved from cached model rather than assumed qpos ordering.
            import mujoco
            model = mujoco.MjModel.from_binary_path(str(c.case_dir(case_id)/c.STAGES[0]/'compiled_model.mjb')) if not hasattr(background, 'model') else background.model
            background.model = model
            indices = [int(model.jnt_qposadr[model.joint('robot_0/'+n).id]) for n in ('base_x', 'base_y')]
            path = z['qpos'][:, indices]
            ax.plot(path[:, 0], path[:, 1], color=COLORS[r['status']], alpha=.35, lw=.8)
            ax.scatter(*path[-1], marker='^', s=18, color=COLORS[r['status']])
    ax.legend(loc='upper left', fontsize=9)
    ax.set_title('MolmoBot visual mobile Pick | one trial per XY | start dots, measured paths, end triangles')
    fig.tight_layout()
    fig.savefig(figures/'start_results.svg')
    fig.savefig(figures/'start_results.png', dpi=160)
    plt.close(fig)
    trace_rows = [r for r in rows if r.get('attempt') and (folder/'trials'/r['attempt']/'states.npz').exists()]
    if trace_rows:
        cols = 5
        fig, axs = plt.subplots(int(np.ceil(len(trace_rows)/cols)), cols, figsize=(18, 3.6*np.ceil(len(trace_rows)/cols)), squeeze=False)
        for ax, r in zip(axs.ravel(), trace_rows):
            background(ax)
            z = np.load(folder/'trials'/r['attempt']/'states.npz')
            model = background.model
            indices = [int(model.jnt_qposadr[model.joint('robot_0/'+n).id]) for n in ('base_x', 'base_y')]
            path = z['qpos'][:, indices]
            ax.plot(path[:, 0], path[:, 1], color=COLORS[r['status']], lw=1.6)
            ax.scatter(*path[0], marker='o', color=COLORS[r['status']])
            ax.scatter(*path[-1], marker='^', color=COLORS[r['status']])
            ax.set_title(r['pose_id']+' | '+r['reason'], fontsize=8)
            ax.tick_params(labelsize=7)
        for ax in axs.ravel()[len(trace_rows):]:
            ax.axis('off')
        fig.tight_layout()
        fig.savefig(figures/'measured_routes.svg')
        fig.savefig(figures/'measured_routes.png', dpi=130)
        plt.close(fig)
    source = "from pathlib import Path\nimport sys\nsys.path.insert(0,str(Path(__file__).resolve().parents[5]/'src'))\nfrom lastmile.workflow.vla_station_report import export_vla_report\nexport_vla_report("+repr(case_id)+','+repr(station_run_id)+','+repr(run_id)+")\n"
    (figures/'start_results.py').write_text(source)
    fields = ['pose_id', 'status', 'reason', 'initial_base', 'final_base', 'sim_duration_s', 'policy_steps', 'binary_pick_label', 'attempt', 'torso_feedback_warnings', 'collision_warnings']
    with open(report/'results.csv', 'w') as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    baseline = c.read(c.case_dir(case_id)/c.STAGES[2]/'runs'/station_run_id/'physics.json')
    exact = {}
    for s in starts:
        matching = [b for b in baseline if b['pose_id'] == s['pose_id'] and np.allclose(b['base'], s['base'], atol=1e-10, rtol=0)]
        exact[s['pose_id']] = ', '.join(b['revalidated_verdict']['status'] for b in matching) or '未实测该位姿'
    reasons = Counter(r['reason'] for r in rows if r['status'] == 'executed_failure')
    relative_run = os.path.relpath(folder, report)
    lines = [f'# Case1 MolmoBot 视觉移动抓取：{run_id}', '',
        f'**运行状态：{state["status"]}。58 个 XY 起点中，成功 {counts["verified_success"]}、物理失败 {counts["executed_failure"]}、无效初始化 {counts["invalid_initialization"]}、基础设施异常 {counts["infrastructure_error"]}、未运行 {counts["not_run"]}。**', '',
        '每个有效起点单次试验；这不是每点成功概率，也不是固定底盘可抓取性。旧 case1 交付和人工门禁未修改。', '',
        '[TOC]', '', '## 执行设置与事先预期', '',
        '- 本地 MolmoBot-RBY1Multitask，Pick 模式；右腕／头部／左腕 RGB 和22维本体状态，语言指令 `pick up the cup`。目标世界坐标、oracle 抓法和导航路径不输入策略。',
        '- 完整20维动作，头部固定；10 Hz 策略，20 ms 控制周期，4 ms 物理采样；16步预测、8步执行，最多40秒。seed=0，动作限幅有原始输出与执行目标记录。',
        '- 每次恢复原快照，仅改变底盘；A 保留原朝向，其余正对目标。碰撞初始化跳过；原抓取距离界外点不筛除。',
        '- 预期覆盖58点、约35次有效试验；抓取结果事先未知。做成标准是完成有效视觉输入到真实控制和可重放物理验收的证据链，不是预设必须成功。',
        '- 抓取成功仍要求同一夹爪双指真实受力、抬升≥5 cm、无支撑连续保持≥2 s、满足本次碰撞协议，头部固定、实际关节硬限位合规。',
        ('- 本次按用户授权采用躯干反馈告警协议：每步派发目标严格验证 `[0,h,-2h,h,0,0]` 且 h∈[0,0.738]；实际联动残差／范围偏移只警告，不结束 episode。此项与旧严格反馈协议不同，不能作等协议比较。' if feedback_policy == 'warn' else '- 躯干实际联动与范围偏移按原严格协议终止。'),
        (f'- 本次轻微碰撞仅警告并继续；任何机器人接触穿透深度达到 {protocol.get("severe_penetration_m", .02)*100:.1f} cm 时终止为 `severe_penetration`。该阈值是工程默认，不表示真实机器人安全保证；初始化碰撞筛除保持不变。' if collision_policy == 'warn_minor' else '- 碰撞仍按原严格协议终止。'),
        f'- 轻微碰撞告警试验 {len(collision_rows)}，类别采样累计 {collision_warning_samples}，最大告警穿透 {max_penetration*1000:.3f} mm；每个接触对／深度／受力均有原始记录。',
        f'- 躯干反馈策略：`{feedback_policy}`；有警告试验 {len(warning_rows)}，警告类别采样累计 {warning_samples}，最大残差 {max_residual:.6f} rad（原始4 ms轨迹可重算）。', '',
        '## 起点结果与实际路线', '',
        '![起点结果与实测底盘轨迹](figures/start_results.png)', '',
        '图：观测单位为一个 XY 起点的单次试验。圆点为起点，三角为实测终点，叉为无效初始化；灰色障碍投影仅用于显示。没有误差棒，不把未运行或无效点填成失败。', '']
    if trace_rows:
        lines += ['![逐点实测路线](figures/measured_routes.png)', '', '图：各子图给出独立恢复后连续执行的实测底盘路线；不是规划路线。', '']
    lines += ['## 逐点证据与旧 oracle（仅描述性并列）', '',
              '| 起点 | VLA状态 | 原因 | 旧固定底盘相同位姿 | 视频 |', '|---|---|---|---|---|']
    for r in rows:
        video = f'[视频]({relative_run}/trials/{r["attempt"]}/rollout.mp4)' if r.get('attempt') else '—'
        lines.append(f'| {r["pose_id"]} | {r["status"]} | {r["reason"]} | {exact[r["pose_id"]]} | {video} |')
    lines += ['', '## 结果边界与下一步', '', f'- 物理失败原因计数：`{dict(reasons)}`。这是事实分类，不是已验证的因果归因。',
        '- baseline 是固定底盘、冻结 grasp 的 oracle，与本次允许移动且双臂可动的 VLA 不等条件；未做整体胜率或显著性比较。',
        '- 每点只有一次且全部使用同一模型与seed，不做概率、独立样本或泛化声明。原始相机／模型输入、模型动作、限幅、物理接触和轨迹均可追溯。',
        '- 无效初始化不支持判断模型能力；基础设施异常／未运行点不支持判断物理失败。若批次中断，修复公共接口后必须按冻结配置续跑；代码变化需新run。',
        '- 尚未用对照干预验证失败机制；本次仅按用户明确授权更改躯干与轻微碰撞处理，抓取抬升和保持阈值未改变。下一步是否重复试验或更换协议，应另行决定。', '',
        '## 证据与验证', '',
        f'- [冻结来源与配置]({relative_run}/frozen_inputs.json) · [VLA协议]({relative_run}/vla_protocol.json) · [逐点结果]({relative_run}/results.json) · [运行状态]({relative_run}/run_state.json)。',
        '- 每个完成试验的 `audit.json` 从4 ms压缩原始轨迹重算结论；`evidence_manifest.json` 保存输入、动作、轨迹、视频与快照摘要。续跑检查摘要和重新验收。',
        '- 保留：证据图PNG/SVG及复现脚本、紧凑CSV、冻结配置、动作和轨迹。权重只读引用；模型输入、快照、视频与完整日志不加入Git。',
        '- 视觉规划：保留起点结果图和逐点实测路线图（证据类），不采用平滑概率热图或装饰性统计图。HTML使用同一Markdown生成，图片内嵌，可离线阅读。', '']
    text = '\n'.join(lines)
    (report/'REPORT.md').write_text(text)
    body = markdown.markdown(text, extensions=['tables', 'toc'])
    for name in ('start_results.png', 'measured_routes.png'):
        if (figures/name).exists():
            uri = 'data:image/png;base64,'+base64.b64encode((figures/name).read_bytes()).decode()
            body = body.replace('src="figures/'+name+'"', 'src="'+uri+'"')
    (report/'REPORT.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>Case1 MolmoBot VLA</title><style>body{max-width:1200px;margin:32px auto;padding:0 24px;font:16px/1.65 sans-serif;color:#25313a}img{max-width:100%;cursor:zoom-in}table{border-collapse:collapse;font-size:13px}td,th{padding:7px;border-bottom:1px solid #ddd;text-align:left}h2{margin-top:36px}a{color:#236ba0}@media print{body{margin:0}a{color:inherit}}</style>'+body+'</html>')
    c.write(report/'report_checks.json', {'images_embedded_in_html': True, 'image_files_exist': all((figures/n).exists() for n in ('start_results.png', 'start_results.svg')),
        'visual_inspection': 'pending_manual_or_browser_check', 'report_generated_from_markdown': True})
    return summary
