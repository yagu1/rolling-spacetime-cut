"""Render toy-map trajectories, safe holding, restriction gates, and batch results.

Usage: python experiments/render_contingency_restriction.py
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from experiments.contingency_restriction import make_contingency_bridge, SCENARIOS

OUT=ROOT/'output'/'contingency_restriction'


def load(sc,mode):
    return json.loads((OUT/f'{sc}_{mode}.json').read_text(encoding='utf-8'))


def decorate(ax,sc):
    graph,aa=make_contingency_bridge()
    xs=np.array([p[0] for p in graph.cells]);ys=np.array([p[1] for p in graph.cells])
    ax.scatter(xs,ys,s=100,marker='s',c='0.88',edgecolors='0.75',lw=.3,zorder=0)
    for ev in SCENARIOS[sc]:
        ax.scatter([ev.cell[0]],[ev.cell[1]],s=180,c='tab:red',marker='x',lw=2.5,
                   label='Temporary obstacle',zorder=6)
    for a in aa:
        ax.scatter([a.start[0]],[a.start[1]],s=30,c='black',marker='o',zorder=7)
        ax.annotate(a.name,(a.start[0]+0.1,a.start[1]+.05),fontsize=8)
        ax.scatter([a.goal[0]],[a.goal[1]],s=40,c='black',marker='*',zorder=7)
    ax.set_xlim(-1,15);ax.set_ylim(5,-1)
    ax.set_aspect('equal',adjustable='box')
    ax.set_xlabel('Grid x (m)');ax.set_ylabel('Grid y (m)')
    ax.grid(alpha=.18)


def plot_map(sc,mode):
    r=load(sc,mode)
    fig,ax=plt.subplots(figsize=(10,4.2),layout='constrained')
    decorate(ax,sc)
    for n,path in r['traces'].items():
        q=np.asarray(path)
        ax.plot(q[:,0],q[:,1],lw=2.1,alpha=.8,label=f'Rover {n}')
    ax.set_title(f'{sc}: {mode} — independent rover trajectories')
    ax.legend(fontsize=8,loc='lower center',ncol=5)
    p=OUT/f'{sc}_{mode}_routes.png'
    fig.savefig(p,dpi=165);plt.close(fig)
    return p


def plot_progress():
    fig,ax=plt.subplots(figsize=(10.5,4.8),layout='constrained')
    for m,style in [('restriction','-'),('central_path','--')]:
        r=load('approach_detour',m)
        for n in ('C','D'):
            yy=np.asarray(r['traces'][n])[:,0]
            ax.plot(np.arange(len(yy)), yy,ls=style,lw=2,
                    label=f'{n}: {m}')
    ev=SCENARIOS['approach_detour'][0]
    ax.axvspan(ev.active_from,ev.active_until,alpha=.10,
               label='Obstacle active')
    ax.axvline(10,lw=1,ls=':',color='0.4',label='First 10-s supervisor refresh')
    ax.set_title('Unexpected approach obstruction: progress between supervisor updates')
    ax.set_xlabel('Simulation time (s)');ax.set_ylabel('Rover x position (m)')
    ax.set_xlim(0,70);ax.grid(alpha=.2)
    ax.legend(ncol=3,fontsize=8)
    p=OUT/'approach_recovery_time.png';fig.savefig(p,dpi=165);plt.close(fig)
    return p


def plot_hold():
    fig,ax=plt.subplots(figsize=(10.5,4.8),layout='constrained')
    for m,style in [('restriction','-'),('central_path','--')]:
        r=load('bridge_closure',m)
        for n in ('C','D'):
            yy=np.asarray(r['traces'][n])[:,0]
            ax.plot(np.arange(len(yy)),yy,ls=style,lw=2,label=f'{n}: {m}')
    ev=SCENARIOS['bridge_closure'][0]
    ax.axvspan(ev.active_from,ev.active_until,alpha=.12,label='Bridge temporarily blocked')
    ax.set_title('Single-lane bridge obstruction: waiting and resumed movement')
    ax.set_xlabel('Simulation time (s)');ax.set_ylabel('Rover x position (m)')
    ax.set_xlim(0,70);ax.grid(alpha=.2);ax.legend(ncol=3,fontsize=8)
    p=OUT/'bridge_hold_and_resume.png';fig.savefig(p,dpi=165);plt.close(fig)
    return p


def plot_batch():
    data=json.loads((OUT/'seed_sweep.json').read_text(encoding='utf-8'))['summary']
    scs=['none','approach_detour','bridge_closure']
    modes=['restriction','central_path']
    means={(a['scenario'],a['method']):a['mean_completion_s_successful_only'] for a in data}
    stds={(a['scenario'],a['method']):a['std_completion_s_successful_only'] for a in data}
    fig,ax=plt.subplots(figsize=(9.5,4.8),layout='constrained')
    x=np.arange(3);w=.35
    for k,mode in enumerate(modes):
        yy=[means[(sc,mode)] for sc in scs]
        ee=[stds[(sc,mode)] for sc in scs]
        ax.bar(x+(k-.5)*w,yy,w,yerr=ee,capsize=4,label=mode)
    ax.set_xticks(x, ['No event','Approach obstruction','Bridge closure'])
    ax.set_ylim(0,max(v for v in means.values() if v is not None)*1.2)
    ax.set_ylabel('Mean completion time (s) ± sample SD')
    ax.set_title('Preliminary 10-seed pilot (only successful trials; two controlled modes)')
    ax.legend();ax.grid(axis='y',alpha=.15)
    p=OUT/'pilot_seed_sweep.png';fig.savefig(p,dpi=165);plt.close(fig)
    return p


def plot_grants():
    r=load('bridge_closure','restriction')
    fig,ax=plt.subplots(figsize=(11,4),layout='constrained')
    labels=list('ABCD')
    last={n:None for n in labels}
    for z in r['timeline']:
        t=z['time'];windows=z['grant_windows']
        for n in labels:
            w=windows.get(n)
            if w and w['open_at']<=t<w['close_at']:
                ax.broken_barh([(t,1)],(labels.index(n)-.3,.6))
    for n in labels:
        q=np.asarray(r['traces'][n])
        on=[3<=p[0]<=11 and p[1]==2 for p in q]
        ax.scatter([i for i,f in enumerate(on) if f],
                   [labels.index(n)]*sum(on),s=8,edgecolors='black',facecolors='none',lw=.6)
    ax.axvspan(6,23,alpha=.08,label='Bridge blocked')
    ax.set_yticks(range(4), labels)
    ax.set_ylim(-.6,3.6);ax.set_xlim(0,68)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Rover')
    ax.set_title('Published access windows and actual bridge occupancy (hollow circles)')
    ax.grid(axis='x',alpha=.2)
    p=OUT/'bridge_restriction_leases.png';fig.savefig(p,dpi=165);plt.close(fig)
    return p


def animate(sc):
    r=load(sc,'restriction');trace={n:np.asarray(p) for n,p in r['traces'].items()}
    count=len(r['timeline'])
    fig,ax=plt.subplots(figsize=(9,3.9),layout='constrained')
    decorate(ax,sc)
    dots={n:ax.plot([],[],marker='o',linestyle='None',markersize=9,
                    label=f'Rover {n}')[0] for n in sorted(trace)}
    ax.legend(ncol=5,fontsize=8,loc='lower center')
    def update(i):
        for n,art in dots.items():
            p=trace[n][min(i,len(trace[n])-1)]
            art.set_data([p[0]],[p[1]])
        ax.set_title(f'{sc}: restriction-only navigation — t={i} s')
        return list(dots.values())
    ani=FuncAnimation(fig,update,frames=range(0,count+1,1),interval=160,blit=False)
    p=OUT/f'{sc}_restriction.gif';ani.save(str(p),writer=PillowWriter(fps=5))
    plt.close(fig)
    return p


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    outputs=[]
    for sc in ('approach_detour','bridge_closure'):
        for mode in ('restriction','central_path'):
            outputs.append(plot_map(sc,mode))
        outputs.append(animate(sc))
    outputs.extend([plot_progress(),plot_hold(),plot_batch(),plot_grants()])
    for p in outputs:print('Saved',p)

if __name__=='__main__':main()
