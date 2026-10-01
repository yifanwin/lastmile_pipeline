"""从完整原始闭环轨迹生成实际路线及 Pick 抬升证据图。"""
import numpy as np
from . import core as c

def plot_closed_loop(case_id,run_id):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    root=c.case_dir(case_id)/c.STAGES[4]/'runs'/run_id;s=c.read(root/'summary.json');config=c.read(root/'frozen_inputs.json')['config'];folder=root/'delivery';folder.mkdir(exist_ok=True)
    fig,(ax,bx)=plt.subplots(1,2,figsize=(12,5));center=np.array(config['table_center']);size=np.array(config['table_size'])
    ax.add_patch(plt.Rectangle(center[:2]-size[:2]/2,*size[:2],color='#ead7b9',ec='#8a785f',label='Table footprint'))
    colors=['#1479b8','#c07721','#21835d']
    for i,result in enumerate(s['trials']):
        trial=root/f'trial_{result["trial"]:02d}';nav=c.read(trial/'navigation_trace.json')['samples'];xy=np.array([r['base'][:2] for r in nav]);ax.plot(xy[::10,0],xy[::10,1],color=colors[i],lw=1.6,label=f'Measured trial {i}')
        p=c.read(trial/'pick/grasp_0794_trace.json');t=np.array([r['time_s'] for r in p['samples']]);height=np.array([r['height_m'] for r in p['samples']])-p['initial']['height_m'];bx.plot(t-t[0],100*height,color=colors[i],label=f'Trial {i}: {result["status"]}',lw=1.3)
    for i,path in enumerate(config['navigation_segments']):
        p=np.array(path);ax.plot(p[:,0],p[:,1],'--',color='#555',lw=1,label='A* reference' if i==0 else None)
    for name,key,color,marker in [('P131','start_base','#c84444','o'),('Staging','staging_base','#a076bd','s'),('A','goal_base','#228354','o')]:
        p=config[key];ax.scatter(*p[:2],color=color,marker=marker,s=60,zorder=4);ax.annotate(name,p[:2],xytext=(6,6),textcoords='offset points')
    target=np.array(config['target_pose'])[:2,3];ax.scatter(*target,marker='*',s=130,color='#e89e22',label='Target',zorder=4)
    ax.set(xlabel='World x (m)',ylabel='World y (m)',title='A* reference vs actual continuous navigation',xlim=(6.3,8.45),ylim=(.25,3.1));ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8,loc='lower right')
    bx.axhline(5,color='#b64a44',ls='--',lw=1,label='5 cm lift threshold');bx.set(xlabel='Time since Pick start (s)',ylabel='Target lift (cm)',title='Physical Pick traces (not success by height alone)');bx.grid(alpha=.2);bx.legend(fontsize=8)
    fig.tight_layout()
    for ext in ['png','svg']:fig.savefig(folder/f'closed_loop_evidence.{ext}',dpi=150)
    plt.close(fig);return folder/'closed_loop_evidence.png'
