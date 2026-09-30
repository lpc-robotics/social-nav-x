"""Plot CPU evidence; all distances include both physical robot footprints."""
from pathlib import Path
import csv, json, math
from collections import defaultdict
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
here=Path(__file__).resolve().parent
styles={'legacy_r1':('Legacy kernel: R1 only','#a855f7','--'),'current_r1':('Current: R1 only','#e88416','--'),'current_r2':('Current: R2 only','#3498db','--'),'current_both':('Current: both robots','#15803d','-'),'current_nearest':('Current: nearest only','#777777',':')}
summary={}
for scenario in ['symmetric','staggered','wide']:
    rows=defaultdict(list)
    for row in csv.DictReader((here/scenario/'trajectories.csv').open()):
        mode=row.pop('mode'); rows[mode].append({k:float(v) for k,v in row.items()})
    stats={}
    both=rows['current_both']
    for mode,rs in rows.items():
        diffs=[math.hypot(a['x']-b['x'],a['y']-b['y']) for a,b in zip(rs,both)]
        stats[mode]={'min_clearance_r1_m':min(r['gap_r1'] for r in rs),'min_clearance_r2_m':min(r['gap_r2'] for r in rs),'arrival_sim_s':next((r['t'] for r in rs if r['arrived']),None),'max_same_time_difference_from_both_m':max(diffs),'max_lateral_difference_from_both_m':max(abs(a['y']-b['y']) for a,b in zip(rs,both)),'path_length_m':sum(math.hypot(a['x']-b['x'],a['y']-b['y']) for a,b in zip(rs,rs[1:]))}
    summary[scenario]={'metrics':stats,'simultaneous_force_above_0_01_duration_s':sum(r['force_r1']>.01 and r['force_r2']>.01 for r in both)*.025,'initial_force_r1':both[0]['force_r1'],'initial_force_r2':both[0]['force_r2']}
    fig,axs=plt.subplots(1,3,figsize=(17,5),gridspec_kw={'width_ratios':[1.4,1,1]},layout='constrained')
    ax=axs[0]
    for mode,(label,color,style) in styles.items():
        rs=rows[mode]; ax.plot([r['x'] for r in rs],[r['y'] for r in rs],style,color=color,lw=2,label=label)
    for i,(x,y) in enumerate([(11,9.2),(11.6 if scenario!='symmetric' else 11,11.2 if scenario=='wide' else 10.8)]):
        ax.add_patch(Circle((x,y),.35,color='#334155',alpha=.8)); ax.add_patch(Circle((x,y),.75,fill=False,color='#334155',ls=':',alpha=.7))
        ax.text(x,y,'R'+str(i+1),color='white',ha='center',va='center')
    ax.scatter([10],[10],c='black',marker='o'); ax.scatter([16],[10],c='black',marker='*',s=120)
    ax.set(xlabel='x (m)',ylabel='y (m)',title='Pedestrian center trajectories',xlim=(9.6,16.4),ylim=(8,12)); ax.set_aspect('equal'); ax.legend(fontsize=8,loc='lower right')
    ax=axs[1]
    for mode,(label,color,style) in styles.items():
        rs=rows[mode]; ax.plot([r['t'] for r in rs],[min(r['gap_r1'],r['gap_r2']) for r in rs],style,color=color,lw=1.5,label=label)
    ax.axhline(0,color='red',lw=1,label='Footprint overlap'); ax.axhline(.1,color='black',ls=':',lw=1)
    ax.set(xlabel='Simulation time (s)',ylabel='Minimum clearance to either robot (m)',title='Safety measured against BOTH robots',xlim=(0,12),ylim=(-.6,1.5))
    ax=axs[2]
    for key,color,label in [('force_r1','#e88416','R1 contribution'),('force_r2','#3498db','R2 contribution')]:
        ax.plot([r['t'] for r in both],[r[key] for r in both],color=color,label=label)
    ax.set(xlabel='Simulation time (s)',ylabel='Force-term magnitude (before total clipping)',title='Both contributions in the same timestep',xlim=(0,12)); ax.legend(fontsize=8)
    fig.suptitle(('Symmetric bottleneck: simultaneous response can stall' if scenario=='symmetric' else ('Wider staggered gap: completed avoidance' if scenario=='wide' else 'Staggered narrow gap: stalled response'))+'\nCPU dynamics only; fixed robots; original kernel baseline is not a full Isaac run',fontsize=12)
    fig.savefig(here/scenario/'comparison.png',dpi=160); fig.savefig(here/scenario/'comparison.pdf'); plt.close(fig)
(here/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
