"""Render state-dependent XYT cuts and clearance planning diagnostics."""
import json,sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
r=json.loads((ROOT/'output/state_margin/report.json').read_text())
out=ROOT/'output/state_margin';out.mkdir(exist_ok=True,parents=True)
colors={'A':'tab:blue','B':'tab:green','C':'tab:orange','D':'tab:purple'}
fig,axes=plt.subplots(2,4,figsize=(15,6),sharex=True,sharey=True)
for row,mode in enumerate(('static','state')):
    frames=r['unseeded_cuts'][mode]['frames']
    for col,t in enumerate((0,7,14,21)):
        ax=axes[row,col]; frame=frames[t]
        for name,pts in frame['regions'].items():
            if pts:
                a=np.asarray(pts);ax.scatter(a[:,0],a[:,1],color=colors[name],s=110,marker='s',alpha=.75)
        for p,q in frame['boundary_edges']:
            ax.plot([p[0],q[0]],[p[1],q[1]],color='black',lw=1,alpha=.5)
        ax.set_aspect('equal');ax.set_xlim(-1,15);ax.set_ylim(4,-0.1)
        ax.set_title(f'{mode} potential, t={t}')
        if col==0:ax.set_ylabel('Y cells')
        if row==1:ax.set_xlabel('X cells')
fig.suptitle('Unseeded 4-label cuts: dynamic unary costs do not imply directed connectivity')
fig.tight_layout();fig.savefig(out/'cut_comparison.png',dpi=155);plt.close(fig)
fig,ax=plt.subplots(figsize=(9,4.5))
record=r['margin_aware_planning']
for i,name in enumerate(('A','B','C','D')):
    xs=[];ys=[]
    for sep,entry in record.items():
        if entry['found']:
            xs.append(float(sep));ys.append(entry['arrival'][name])
    ax.plot(xs,ys,'o-',label=name)
ax.set_xlabel('Required center-to-center separation (grid units)');ax.set_ylabel('Arrival time step');ax.set_title('Four-robot bridge: clearance-constrained prioritized A*')
ax.set_xlim(-.05,1.5);ax.set_ylim(0,30);ax.grid(alpha=.3);ax.legend(ncol=4)
ax.text(.91,24,'No solution found by current\n28-step prioritized search',fontsize=9)
fig.tight_layout();fig.savefig(out/'margin_comparison.png',dpi=160);plt.close(fig)
print('Rendered',out)
