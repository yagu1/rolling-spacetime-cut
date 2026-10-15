"""Render the reachability/margin experimental report into reproducible figures.

Run after `python experiments/reachability_margin_cut.py`.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.colors import ListedColormap
import numpy as np
from PIL import Image, ImageDraw, ImageFont

COLORS={'A':'#2563eb','B':'#10b981','C':'#f97316','D':'#a855f7'}
NAMES=list(COLORS)

def _frames_to_labels(frames):
    return [{tuple(p):n for n,coords in f['regions'].items() for p in coords} for f in frames]


def draw_region(ax,frame,agents,t,paths=None,radius=.28,pair_margin=.34):
    color=COLORS
    ownership={tuple(p):n for n,coords in frame['regions'].items() for p in coords}
    for (x,y),n in ownership.items():
        ax.add_patch(Rectangle((x-.49,y-.49),.98,.98,facecolor=color[n],alpha=.24,edgecolor='#d6dee5',linewidth=.6))
    for p,q in frame.get('boundary_edges',[]):
        x,y=p;xx,yy=q
        mx,my=(x+xx)/2,(y+yy)/2
        if x!=xx: ax.plot([mx,mx],[my-.50,my+.50],color='#161c25',linewidth=1.1,alpha=.75,zorder=3)
        else:ax.plot([mx-.50,mx+.50],[my,my],color='#161c25',linewidth=1.1,alpha=.75,zorder=3)
    if paths:
        for n,seq in paths.items():
            pos=tuple(seq[t])
            px,py=pos
            # Halo represents robot disc + 1/2 symmetric pair-margin.
            ax.add_patch(Circle((px,py),radius+pair_margin/2,facecolor='none',
                                edgecolor=color[n],linewidth=1.4,linestyle='--',zorder=6))
            ax.add_patch(Circle((px,py),radius,facecolor=color[n],alpha=.94,
                                edgecolor='white',linewidth=.75,zorder=7))
            ax.text(px,py,n,ha='center',va='center',color='white',weight='bold',fontsize=7,zorder=8)
    else:
        for n,a in agents.items():
            xx,yy=(a['start'] if t==0 else a['goal'] if t==max(len(frame.get('regions',{})),30) else (-100,-100))
            if xx!=-100:ax.text(xx,yy,n,ha='center',va='center',fontsize=8)
    ax.set_xlim(-.85,14.85);ax.set_ylim(4.15,-.15);ax.set_aspect('equal')
    ax.set_xticks([0,3,7,11,14]);ax.set_yticks([1,2,3]);ax.tick_params(labelsize=8)
    ax.grid(False)


def draw_progress(data,out):
    frames=[data['initial_cut']['frames'],data['directed_feedback_cut']['frames'],
            (data.get('safety_feedback_cut') or {}).get('frames')]
    if frames[2] is None:return
    paths=data['safety_feedback_cut'].get('joint_paths')
    slices=[4,12,20,28]
    fig,axes=plt.subplots(3,len(slices),figsize=(17,9.5),sharex=True,sharey=True)
    titles=['Initial potential-only cut','Directed-reachability feedback','Margin-aware flow feedback']
    for r,fs in enumerate(frames):
        for c,t in enumerate(slices):
            ax=axes[r,c]
            draw_region(ax,fs[t],data['agents'],t,paths if r==2 else None,
                        data['model']['disc_radius_m'],data['model']['pair_extra_margin_m'])
            ax.set_title(f't = {t}',fontsize=11,weight='bold')
            if c==0:ax.set_ylabel(titles[r],fontsize=10,weight='bold')
    fig.suptitle('Four robots / One-cell bridge / XYT ownership from potential and reachability feedback',fontsize=13,weight='bold')
    fig.text(.5,.035,'Colored cells: ownership labels. Dashed circles: robot radius + half pair margin (not certified free-space regions).',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.055,1,.95]);fig.savefig(out/'cut_slice_comparison.png',dpi=170);plt.close(fig)


def draw_xt(data,out):
    stages=[('Initial cut',data['initial_cut']),
            ('Reachability feedback',data['directed_feedback_cut']),
            ('Reachability + margin feedback',data.get('safety_feedback_cut'))]
    stages=[x for x in stages if x[1] is not None]
    fig,axes=plt.subplots(1,len(stages),figsize=(16,9),sharex=True,sharey=True)
    cmap=ListedColormap([COLORS[n] for n in NAMES])
    for ax,(title,d) in zip(axes,stages):
        labs=_frames_to_labels(d['frames'])
        mat=np.array([[NAMES.index(labs[t][(x,2)]) for x in range(3,12)] for t in range(len(labs))])
        ax.imshow(mat,aspect='auto',interpolation='nearest',extent=[2.5,11.5,len(labs)-0.5,-0.5],cmap=cmap,vmin=-.5,vmax=3.5,alpha=.68)
        for t in range(len(mat)):
            for j in range(len(mat[t])-1):
                if mat[t,j]!=mat[t,j+1]:
                    ax.plot([j+3.5,j+3.5],[t-.5,t+.5],color='black',lw=1.15)
        p=d.get('joint_paths')
        if p:
            for n,seq in p.items():
                xx=[seq[t][0] if seq[t][1]==2 and 3<=seq[t][0]<=11 else np.nan for t in range(len(seq))]
                tt=np.arange(len(seq))
                ax.plot(xx,tt,color='white',lw=3,alpha=.92)
                ax.plot(xx,tt,color=COLORS[n],lw=1.8,label=f'{n} path')
        ax.set_title(title,fontsize=12,weight='bold')
        ax.set_xlabel('Position x along bridge',fontsize=10)
        ax.set_xticks([3,5,7,9,11]);ax.grid(False)
    axes[0].set_ylabel('Time step (downward)',fontsize=10)
    fig.suptitle('How the space-time cut boundary changes as directed and safety state is fed back',fontsize=13,weight='bold')
    from matplotlib.patches import Patch
    fig.legend(handles=[Patch(facecolor=COLORS[n],label=f'Robot {n}') for n in NAMES],
               ncol=4,loc='lower center',bbox_to_anchor=(.5,.02),fontsize=9)
    fig.tight_layout(rect=[0,.07,1,.94]);fig.savefig(out/'bridge_xt_boundary.png',dpi=170);plt.close(fig)


def draw_safety(data,out):
    d=data.get('safety_feedback_cut')
    if not d or not d.get('geometry'):return
    g=d['geometry'];tracks=g['per_transition']
    ts=np.array([x['t'] for x in tracks])
    sep=np.array([x['pair_center_distance_m'] for x in tracks])
    clearance=np.array([x['max_shared_path_deviation_m'] for x in tracks])
    fig,axes=plt.subplots(2,1,figsize=(11,6.7),sharex=True)
    need=data['model']['pair_required_center_m']
    axes[0].plot(ts,sep,'o-',markersize=3,color='#2e80c0',label='Minimum swept center distance')
    axes[0].axhline(need,color='#d64b4b',linestyle='--',label=f'Required = {need:.2f} m')
    axes[0].set_ylabel('Pairwise separation (m)');axes[0].grid(alpha=.2);axes[0].legend(loc='upper right')
    axes[0].set_title('Synchronized continuous-time separation of four moving robot discs',weight='bold')
    axes[1].fill_between(ts,0,clearance,step='mid',color='#10b981',alpha=.5)
    axes[1].plot(ts,clearance,'-',color='#0b8874',linewidth=1.4)
    axes[1].set_ylabel('Admissible shared path deviation (m)')
    axes[1].set_xlabel('Transition t to t + 1');axes[1].set_ylim(0,max(.15,max(clearance)*1.25))
    axes[1].grid(alpha=.2)
    fig.text(.5,.008,'Conservative deviation bound limited by pair clearance and swept wall clearance; not arbitrary safety of all cut-labeled cells.',
             ha='center',fontsize=8)
    fig.tight_layout(rect=[0,.035,1,1]);fig.savefig(out/'margin_and_tube.png',dpi=170);plt.close(fig)


def draw_cones(data,out):
    fig,axes=plt.subplots(2,1,figsize=(12,5.8),sharex=True)
    cs=data['reachability_cones']
    for ax,n in zip(axes,['A','C']):
        counts=np.array(cs[n]['per_t']); tt=np.arange(len(counts))
        ax.fill_between(tt,0,counts,color=COLORS[n],alpha=.28)
        ax.plot(tt,counts,color=COLORS[n],lw=2)
        ax.set_ylabel(f'{n}: viable states')
        ax.grid(alpha=.22)
    axes[-1].set_xlabel('Time step')
    fig.suptitle('Single-agent forward/backward reachability: count of XYT states per time layer',weight='bold')
    fig.tight_layout();fig.savefig(out/'viable_cones.png',dpi=170);plt.close(fig)


def make_gif(data,out):
    d=data.get('safety_feedback_cut')
    if not d or 'joint_paths' not in d:return
    paths=d['joint_paths'];fs=d['frames'];agents=data['agents']
    width,height=1120,455
    unit=58;xoff=115;yoff=125
    font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font=ImageFont.truetype(font_path,20) if Path(font_path).exists() else ImageFont.load_default()
    big=ImageFont.truetype(font_path,28) if Path(font_path).exists() else ImageFont.load_default()
    frames=[]
    r=data['model']['disc_radius_m'];halo=r+data['model']['pair_extra_margin_m']/2
    for t,f in enumerate(fs):
        im=Image.new('RGB',(width,height),'#ffffff')
        dr=ImageDraw.Draw(im,'RGBA')
        dr.text((32,30),'State-aware XYT cut + physical margin | 4 robots / one bridge',fill='#182536',font=big)
        dr.text((860,32),f'Time: {t:02d} / {len(fs)-1:02d}',fill='#182536',font=big)
        grid={tuple(p):n for n,pts in f['regions'].items() for p in pts}
        for (x,y),n in grid.items():
            xx=xoff+x*unit;yy=yoff+y*unit
            rgb=tuple(bytes.fromhex(COLORS[n][1:]))
            dr.rectangle((xx-unit/2+1, yy-unit/2+1,xx+unit/2-1,yy+unit/2-1),fill=(*rgb,57),outline='#c8cdd3',width=1)
        for n,seq in paths.items():
            rgb=tuple(bytes.fromhex(COLORS[n][1:]))
            trace=[(xoff+seq[k][0]*unit,yoff+seq[k][1]*unit) for k in range(t+1)]
            if len(trace)>1:dr.line(trace,fill=(*rgb,145),width=4,joint='curve')
            xx,yy=trace[-1]
            dr.ellipse((xx-halo*unit,yy-halo*unit,xx+halo*unit,yy+halo*unit),outline=(*rgb,210),width=3)
            dr.ellipse((xx-r*unit,yy-r*unit,xx+r*unit,yy+r*unit),fill=(*rgb,255),outline='#ffffff',width=2)
            dr.text((xx-8,yy-13),n,fill='white',font=font)
        dr.text((32,378),'Dashed outline on static PNG / outer circle in animation: radius + half safety margin',fill='#334155',font=font)
        dr.text((32,412),'Ownership colors do not certify arbitrary collision-free movement. Only the tested synchronized paths are certified.',fill='#64748b',font=font)
        frames.append(im.convert('P',palette=Image.Palette.ADAPTIVE))
    frames[0].save(out/'four_robot_state_margin.gif',save_all=True,append_images=frames[1:],duration=280,loop=0,optimize=False)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--report',default='output/reachability_margin/report.json')
    ap.add_argument('--output-dir',default='docs/figures/reachability_margin')
    args=ap.parse_args()
    data=json.loads((ROOT/args.report).read_text(encoding='utf8'))
    out=ROOT/args.output_dir;out.mkdir(parents=True,exist_ok=True)
    draw_progress(data,out);draw_xt(data,out);draw_safety(data,out);draw_cones(data,out);make_gif(data,out)
    print('Rendered:',*[str(p) for p in out.iterdir()],sep='\n')

if __name__=='__main__':main()
