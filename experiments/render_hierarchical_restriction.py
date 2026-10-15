"""Generate reproducible figures for the 60s/10s restriction-only demo."""
from __future__ import annotations
import csv
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from experiments.four_agent_bridge import make_four_bridge
from experiments.hierarchical_restriction import dist_to_segment_zero

OUT=ROOT/'output/hierarchical_restriction'


def save_fig(name:str):
    plt.savefig(OUT/name,dpi=175,bbox_inches='tight')
    plt.close()


def run(report=None):
    OUT.mkdir(parents=True,exist_ok=True)
    data=report or json.loads((OUT/'report.json').read_text(encoding='utf8'))
    graph, agents=make_four_bridge(True)
    names=[a.name for a in agents]
    colors=dict(zip(names, plt.rcParams['axes.prop_cycle'].by_key()['color'][:4]))
    duration=data['total_simulated_seconds']
    paths={n:[tuple(p) for p in coords] for n,coords in data['traces'].items()}
    updates=data['supervisor_updates']

    # Fig 1: access windows plus actual resource occupation over the full run.
    fig,axs=plt.subplots(2,1,figsize=(11,6),sharex=True,
                        gridspec_kw={'height_ratios':[2.2,1]},layout='constrained')
    ax=axs[0]
    for idx,n in enumerate(names):
        for k,event in enumerate(updates):
            issued=event['now']
            next_update=updates[k+1]['now'] if k+1<len(updates) else duration
            win=event['windows'].get(n)
            if win is not None:
                a=max(win['open_at'],issued);b=min(win['close_at'],duration)
                if b>a:
                    ax.broken_barh([(a,b-a)],(idx-.26,.52),facecolors=colors[n],alpha=.25,
                                   edgecolor=colors[n],linewidth=.7)
        occupied=[t for t,p in enumerate(paths[n][:duration+1]) if p[1]==2]
        ax.plot(occupied,[idx]*len(occupied),'o',color=colors[n],markersize=3,
                label=f'{n} actual lane occupancy' if idx==0 else None)
        ax.text(duration+1,idx,str(data['arrivals'][n])+' s',fontsize=9,va='center')
    for event in updates:ax.axvline(event['now'],ls='--',alpha=.2,color='gray')
    ax.set_yticks(range(len(names)),names)
    ax.set_ylabel('Rover')
    ax.set_title('Restriction grant windows (bands) vs. rover-selected lane use (dots)')
    ax.grid(alpha=.14,axis='x')
    ax=axs[1]
    xs=[e['now'] for e in updates]
    ax.bar(xs,[e['search_seconds']*1e3 for e in updates],width=5,alpha=.55,label='MC search runtime')
    ax.set_ylabel('Supervisor ms')
    ax.set_xlabel('Time (s), updates every 10 s')
    ax.set_xlim(-1,duration+7)
    ax.grid(alpha=.14,axis='y')
    save_fig('restriction_schedule.png')

    # Fig 2: true shared map, same positions, dynamically allowed resource.
    snap_times=[3,11,23,31]
    fig,axs=plt.subplots(1,4,figsize=(13,4),sharex=True,sharey=True,layout='constrained')
    maps=[]
    for ax,t in zip(axs,snap_times):
        event=max((e for e in updates if e['now']<=t),key=lambda e:e['now'])
        owners=[n for n,w in event['windows'].items() if w['open_at']<=t<w['close_at']]
        for x,y in graph.cells:
            ax.add_patch(plt.Rectangle((x-.5,y-.5),1,1,facecolor='#e5e8ec',edgecolor='white',lw=.6))
            if y==2:
                alpha=.30 if owners else .06
                col='#488bb0' if owners and owners[0] in ('A','B') else '#dd9951'
                ax.add_patch(plt.Rectangle((x-.5,y-.5),1,1,facecolor=col,alpha=alpha,
                                           edgecolor='white',lw=.6))
        for n in names:
            p=paths[n][min(t,len(paths[n])-1)]
            ax.scatter(p[0],p[1],s=110,c=colors[n],edgecolors='black',linewidths=.5,zorder=5)
            ax.text(p[0],p[1],n,ha='center',va='center',color='white',fontsize=9,weight='bold',zorder=6)
        ax.set_title(f't={t}s\nlane: {"/".join(owners) or "closed"}',fontsize=10)
        ax.set_xlim(-.8,14.8);ax.set_ylim(3.7,.3)
        ax.set_aspect('equal');ax.set_xticks([0,3,7,11,14]);ax.set_yticks([1,2,3])
    fig.suptitle('Only the common corridor is restricted; bank cells remain shared',fontsize=12)
    save_fig('restriction_slices.png')

    # Fig 3: measured continuous swept-center clearance and repulsive potential.
    dmins=[];rep=[]
    for t in range(1,duration+1):
        dlist=[]
        for i,a in enumerate(names):
            for b in names[i+1:]:
                dlist.append(dist_to_segment_zero(paths[a][t-1],paths[a][t],paths[b][t-1],paths[b][t]))
        dmin=min(dlist);dmins.append(dmin)
        rep.append(sum(math.exp(-.5*(max(0,d-.9)/.65)**2) for d in dlist))
    fig,axs=plt.subplots(2,1,figsize=(11,6),sharex=True,layout='constrained')
    axs[0].plot(range(1,duration+1),dmins,lw=1.8,label='Minimum continuous swept separation')
    axs[0].axhline(.9,ls='--',color='darkred',label='Required 0.90 m')
    axs[0].set_ylabel('Center distance (m)');axs[0].legend();axs[0].grid(alpha=.2)
    axs[1].plot(range(1,duration+1),rep,lw=1.8,label='Soft repulsion in supervisor candidate scoring')
    axs[1].set_xlabel('Simulation time (s)');axs[1].set_ylabel('Repulsive cost');axs[1].grid(alpha=.2)
    save_fig('swept_clearance_and_repulsion.png')

    # Fig 4: transparent control comparison. Independent shortest paths are
    # individually optimal but NOT a valid multi-robot solution (overlap).
    baseline=data['unrestricted_baseline']
    fig,axs=plt.subplots(1,2,figsize=(9,3.6),layout='constrained')
    x=np.arange(len(names))
    vals0=[baseline['arrival_time_s'][n] for n in names]
    vals1=[data['arrivals'][n] for n in names]
    axs[0].bar(x-.17,vals0,width=.34,label='Unrestricted (unsafe)')
    axs[0].bar(x+.17,vals1,width=.34,label='Restriction-only')
    axs[0].set_xticks(x,names);axs[0].set_ylabel('Arrival time (s)')
    axs[0].legend(fontsize=8);axs[0].grid(alpha=.15,axis='y')
    axs[0].set_title('Unrestricted routes collide')
    axs[1].bar(['Unrestricted','Restriction-only'],
               [baseline['minimum_swept_center_distance_m'],data['minimum_swept_distance_m']])
    axs[1].axhline(data['settings']['minimum_center_distance_m'],ls='--',c='darkred',label='Hard clearance')
    axs[1].set_ylabel('Minimum swept-center distance (m)')
    axs[1].set_title('Safety margin in this deterministic test')
    axs[1].legend(fontsize=8);axs[1].grid(alpha=.15,axis='y')
    save_fig('baseline_comparison.png')

    # Per update runtime + normalized candidate resource coverage.
    with (OUT/'update_summary.csv').open('w',newline='',encoding='utf8') as f:
        writer=csv.DictWriter(f,fieldnames=['at_s','version','evaluated','feasible','runtime_ms','score','predicted_min_sep_m','competing_roi_cells'])
        writer.writeheader()
        for u in updates:
            writer.writerow({'at_s':u['now'],'version':u['version'],'evaluated':u['candidates'],
                             'feasible':u['feasible_candidates'],'runtime_ms':round(u['search_seconds']*1000,3),
                             'score':round(u['score'],4),
                             'predicted_min_sep_m':round(u['predicted_min_swept_distance_m'],4),
                             'competing_roi_cells':u['scope']['competing_bottleneck_cells']})

    # Animation with dynamic restriction colors, no rover route sent by supervisor.
    imgs=[]
    fig,ax=plt.subplots(figsize=(10,3.1),layout='constrained')
    for t in range(duration+1):
        ax.clear()
        event=max((e for e in updates if e['now']<=t),key=lambda e:e['now'])
        owners=[n for n,w in event['windows'].items() if w['open_at']<=t<w['close_at']]
        for x,y in graph.cells:
            col='#ebedef'
            if y==2:
                col='#b5d5e7' if owners and any(n in ('A','B') for n in owners) else '#f4cfab' if owners else '#d3d4d6'
            ax.add_patch(plt.Rectangle((x-.48,y-.48),.96,.96,facecolor=col,edgecolor='white',lw=.7))
        for n in names:
            p=paths[n][t]
            ax.scatter(p[0],p[1],s=185,color=colors[n],edgecolor='black',lw=.6,zorder=3)
            ax.text(p[0],p[1],n,ha='center',va='center',color='white',weight='bold',zorder=4)
        ax.set_xlim(-.7,14.7);ax.set_ylim(3.65,.35)
        ax.set_aspect('equal');ax.set_xticks(range(0,15,2));ax.set_yticks([1,2,3])
        ax.set_title(f'Restriction-only supervision, t={t}s, allow bridge: {"/".join(owners) or "none"}',fontsize=10)
        fig.canvas.draw()
        rgba=np.asarray(fig.canvas.buffer_rgba()).copy()
        imgs.append(Image.fromarray(rgba[:,:,:3]))
    plt.close(fig)
    imgs[0].save(OUT/'four_rovers_restrictions.gif',save_all=True,append_images=imgs[1:],
                 duration=160,loop=0,optimize=True)
    return ['restriction_schedule.png','restriction_slices.png','swept_clearance_and_repulsion.png',
            'baseline_comparison.png',
            'update_summary.csv','four_rovers_restrictions.gif']

if __name__=='__main__':
    print('\n'.join(run()))
