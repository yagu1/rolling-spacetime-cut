"""Computed 4-agent XYT partition figures, animation, and standalone slider.

Run after: python experiments/four_agent_bridge.py
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from PIL import Image,ImageDraw,ImageFont

COLORS={'A':'#2078b4','B':'#27a69a','C':'#de8427','D':'#8555b2'}
BG='#f5f7fa';WALL='#dce2ea';INK='#243141'


def coords(seq):return [tuple(p) for p in seq]


def ownership(frame):
    return {tuple(p):k for k,ps in frame['regions'].items() for p in ps}


def pcolor(hexcolor,alpha=.63):
    return hexcolor


def draw_xy(ax, report, t:int, *, title:bool=True, legend:bool=False):
    cells={tuple(p) for p in report['grid']}
    frame=report['multiway_cut']['frames'][t]
    owner=ownership(frame)
    for x in range(15):
        for y in range(1,4):
            p=(x,y)
            ax.add_patch(Rectangle((x-.5,y-.5),1,1,
                         facecolor=(COLORS[owner[p]] if p in cells else WALL),
                         edgecolor='white',lw=.9,alpha=.85 if p in cells else .5))
    for p,q in frame['boundary_edges']:
        x1,y1=p;x2,y2=q
        if x1!=x2:ax.plot([min(x1,x2)+.5]*2,[y1-.5,y1+.5],color='#172333',lw=2.5)
        else:ax.plot([x1-.5,x1+.5],[min(y1,y2)+.5]*2,color='#172333',lw=2.5)
    for n,data in report['agents'].items():
        path=report['schedule']['paths'][n]
        x,y=path[t]
        ax.scatter([x],[y],s=225,c=COLORS[n],edgecolors='white',lw=2.0,zorder=5)
        ax.text(x,y,n,ha='center',va='center',color='white',size=11,weight='bold',zorder=6)
        sx,sy=data['start'];gx,gy=data['goal']
        ax.plot([sx],[sy],ls='None',marker='s',mfc='None',mec=COLORS[n],ms=10,mew=1.8,zorder=4)
        ax.plot([gx],[gy],ls='None',marker='x',c=COLORS[n],ms=9,mew=2,zorder=4)
    ax.set(xlim=(-.6,14.6),ylim=(3.6,.4),aspect='equal',xticks=range(0,15,2),yticks=[1,2,3])
    ax.tick_params(labelsize=8)
    if title:ax.set_title(f't = {t:02d}  |  XY Cut boundary',fontsize=12)
    if legend:ax.legend(handles=[Line2D([0],[0],marker='o',c='white',markerfacecolor=COLORS[n],
                                markersize=9,label=f'{n} ({"L→R" if n in ("A","B") else "R→L"})')
                             for n in COLORS],ncol=4,loc='upper center',bbox_to_anchor=(.5,-.12),frameon=False)


def bridge_xt(report,out:Path):
    H=report['horizon'];names=list(COLORS)
    mats=[]
    for fr in report['multiway_cut']['frames']:
        owners=ownership(fr);mats.append([names.index(owners[(x,2)]) for x in range(3,12)])
    arr=np.array(mats)
    fig,ax=plt.subplots(figsize=(10.6,9.1))
    ax.imshow(arr,origin='upper',extent=(2.5,11.5,H+.5,-.5),aspect='auto',
              cmap=ListedColormap([COLORS[k] for k in names]),vmin=-.5,vmax=3.5,alpha=.75)
    for t in range(H+1):
        for j in range(8):
            if arr[t,j]!=arr[t,j+1]:ax.plot([3+j+.5]*2,[t-.5,t+.5],color='#15202a',lw=2.1)
            if t and arr[t,j]!=arr[t-1,j]:ax.plot([3+j-.5,3+j+.5],[t-.5]*2,color='#15202a',lw=1.2)
        if t and arr[t,8]!=arr[t-1,8]:ax.plot([10.5,11.5],[t-.5]*2,color='#15202a',lw=1.2)
    markers=['o','D','^','s']
    for j,n in enumerate(names):
        pos=report['schedule']['paths'][n]
        tt=[t for t,p in enumerate(pos) if 3<=p[0]<=11 and p[1]==2]
        xx=[pos[t][0] for t in tt]
        ax.plot(xx,tt,color='white',lw=3.8,alpha=.95)
        ax.plot(xx,tt,color=COLORS[n],lw=2.0,label=f'{n} trajectory',ls='--')
        ax.scatter(xx,tt,edgecolors='#1d2730',facecolors=COLORS[n],marker=markers[j],s=23,lw=.7,zorder=4)
    ax.set(xlabel='Bridge position x (one-cell-wide lane y = 2)',ylabel='Time step ↓',
           xticks=range(3,12),yticks=range(0,H+1,2),ylim=(H+.5,-.5))
    ax.set_title('Four-agent single bridge | four-label XYT Cut boundaries',fontsize=14,weight='bold')
    ax.legend(loc='upper left',bbox_to_anchor=(1.005,1),frameon=False)
    fig.text(.12,.025,'Black segments: cut boundary between neighboring labels. Curves: computed collision-free witness paths on bridge.\nPaths are fixed as hard seeds before the approximate multi-label cut.',fontsize=9,color='#58626e')
    fig.subplots_adjust(left=.13,right=.77,top=.93,bottom=.11)
    fig.savefig(out,dpi=180);plt.close(fig)


def slices(report,out:Path):
    H=report['horizon'];times=[0,4,8,12,16,20,24,H]
    fig,axes=plt.subplots(4,2,figsize=(14,9),constrained_layout=True)
    for t,ax in zip(times,axes.flat):draw_xy(ax,report,t)
    fig.suptitle('Four-agent space–time region allocation | XY time slices',fontsize=15,weight='bold')
    fig.savefig(out,dpi=145);plt.close(fig)


def trajectory_3d(report,out:Path):
    fig=plt.figure(figsize=(10,8));ax=fig.add_subplot(projection='3d')
    for p in report['grid']:
        ax.scatter(p[0],p[1],0,s=7,c='#a5afb8',alpha=.23)
    for n in COLORS:
        path=report['schedule']['paths'][n]
        xs=[p[0] for p in path];ys=[p[1] for p in path];ts=list(range(len(path)))
        ax.plot(xs,ys,ts,c=COLORS[n],lw=3,label=f'{n} (arrival {report["schedule"]["arrival_steps"][n]})')
        ax.scatter(xs[0],ys[0],0,c=COLORS[n],s=80,edgecolor='black',marker='o')
        g=report['agents'][n]['goal']; t0=report['agents'][n]['independent_steps']
        ax.plot([g[0]]*2,[g[1]]*2,[t0,report['horizon']],color=COLORS[n],ls=':',lw=1,alpha=.45)
        ax.scatter(g[0],g[1],report['horizon'],c=COLORS[n],marker='X',s=90)
    ax.set(xlabel='X',ylabel='Y',zlabel='t (step)',zlim=(0,report['horizon']))
    ax.view_init(elev=23,azim=-52)
    ax.set_title('Directed Start → Goal witnesses in XYT (4 agents)',fontsize=14)
    ax.legend(loc='upper left',bbox_to_anchor=(.05,.97))
    fig.savefig(out,dpi=165,bbox_inches='tight');plt.close(fig)


def bridge_timeline(report,out:Path):
    fig,ax=plt.subplots(figsize=(12,4.2))
    H=report['horizon']
    for i,n in enumerate(COLORS):
        path=report['schedule']['paths'][n]
        idx=[t for t,p in enumerate(path) if 3<=p[0]<=11 and p[1]==2]
        if idx:
            ax.broken_barh([(min(idx),max(idx)-min(idx)+1)],(i-.32,.64),facecolors=COLORS[n],alpha=.82)
        for t in idx:ax.scatter(t,i,c='white',edgecolors=COLORS[n],s=15,marker='s',zorder=2)
        a=report['schedule']['arrival_steps'][n]
        ax.text(a+.15,i,f'goal {a}',va='center',fontsize=10)
    ax.set(xlim=(-.5,H+2),ylim=(-.8,3.8),yticks=range(4),yticklabels=list(COLORS),
           xticks=range(0,H+1,2),xlabel='Step',title='Actual occupancy of the single-lane bridge (witness paths)')
    ax.invert_yaxis();ax.grid(axis='x',color='#e6eaf0')
    ax.set_axisbelow(True)
    fig.tight_layout();fig.savefig(out,dpi=170);plt.close(fig)


def font(size,bold=False):
    candidates=['/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf' if bold else '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
                'C:/Windows/Fonts/arialbd.ttf' if bold else 'C:/Windows/Fonts/arial.ttf']
    for p in candidates:
        try:return ImageFont.truetype(p,size)
        except OSError:pass
    return ImageFont.load_default()


def gif_bridge(report,out:Path):
    scale=47; ox=45;oy=116;W=870;Ht=380
    frames=[]
    for frame in report['multiway_cut']['frames']:
        t=frame['t'];owners=ownership(frame)
        im=Image.new('RGB',(W,Ht),'white');d=ImageDraw.Draw(im)
        d.text((30,22),f'4 ROBOTS / ONE BRIDGE     t = {t:02d} / {report["horizon"]}',fill=INK,font=font(22,True))
        d.text((30,62),'Four-label partition (path-seeded), directed witnesses and moving cut lines',fill='#627083',font=font(14))
        for x in range(15):
            for y in range(1,4):
                px=ox+x*scale;py=oy+(y-1)*scale
                p=(x,y)
                col=COLORS[owners[p]] if p in owners else WALL
                d.rectangle((px,py,px+scale-2,py+scale-2),fill=col if p in owners else '#e5eaf0',outline='white',width=1)
        for p,q in frame['boundary_edges']:
            x1,y1=p;x2,y2=q
            if x1!=x2:
                xx=ox+(min(x1,x2)+1)*scale-1
                py=oy+(y1-1)*scale
                d.line((xx,py,xx,py+scale-2),fill=INK,width=3)
            else:
                yy=oy+(min(y1,y2))*scale-1
                xx=ox+x1*scale
                d.line((xx,yy,xx+scale-2,yy),fill=INK,width=3)
        for n,data in report['agents'].items():
            p=report['schedule']['paths'][n][t]
            x,y=p;cx=ox+x*scale+scale//2;cy=oy+(y-1)*scale+scale//2
            d.ellipse((cx-17,cy-17,cx+17,cy+17),fill=COLORS[n],outline='white',width=3)
            d.text((cx-8,cy-12),n,font=font(18,True),fill='white')
        d.line((ox,305,ox+14*scale,305),fill='#b8c4ce',width=2)
        d.line((ox,305,ox+14*scale*t//report['horizon'],305),fill='#223446',width=7)
        for i,n in enumerate(COLORS):
            x=32+i*194
            d.rectangle((x,331,x+17,348),fill=COLORS[n])
            d.text((x+24,329),f'{n}: Goal {report["schedule"]["arrival_steps"][n]}',fill=INK,font=font(14))
        frames.append(im)
    out.parent.mkdir(parents=True,exist_ok=True)
    frames[0].save(out,save_all=True,append_images=frames[1:],duration=240,loop=0,optimize=True)


def offline_explorer(report,out:Path):
    safe=json.dumps({'grid':report['grid'],'frames':report['multiway_cut']['frames'],
            'paths':report['schedule']['paths'],'agents':report['agents'],
            'horizon':report['horizon'],'arrivals':report['schedule']['arrival_steps']},
            ensure_ascii=False).replace('</','<\\/')
    content='''<!doctype html><html lang="en"><meta charset="utf-8"><title>Four-agent XYT cut: time slider</title>
<style>body{font:16px system-ui,sans-serif;margin:25px;color:#243141}header{max-width:1050px}canvas{border:1px solid #d7dee5;max-width:100%}input{width:min(800px,90vw)}.legend{display:flex;gap:24px;margin:12px 0}.legend span{display:inline-block;width:14px;height:14px;vertical-align:middle;margin-right:7px}.notes{max-width:900px;color:#687687;font-size:14px}</style>
<header><h2>4 robots / one-lane bridge — XYT cut boundary</h2>
<p>Time-layer labels from the computed path-seeded four-label alpha-expansion. White-letter dots are robots; thick black edges are boundaries. <strong>This is not a pure graph-cut routing result.</strong></p></header>
<div class="legend" id="legend"></div><input type="range" id="slider" min="0" value="0" step="1"><b id="step"></b>
<div><canvas id="map" width="940" height="270"></canvas></div>
<p class="notes">Squares at agent start locations, crosses at goals; witness routes are hard pinned to preserve directed Start–Goal paths. No time-to-time swept-volume collision safety is asserted.</p>
<script>const data=__DATA__;const palette={A:'#2078b4',B:'#27a69a',C:'#de8427',D:'#8555b2'};
const c=document.getElementById('map'),ctx=c.getContext('2d'),r=document.getElementById('slider');r.max=data.horizon;
document.getElementById('legend').innerHTML=Object.entries(palette).map(([k,v])=>`<div><span style="background:${v}"></span>${k}: ${data.arrivals[k]} steps</div>`).join('');
const sz=48,ox=36,oy=54;const pt=(x,y)=>[ox+x*sz+sz/2,oy+(y-1)*sz+sz/2];
function draw(t){ctx.clearRect(0,0,c.width,c.height);ctx.fillStyle='#fff';ctx.fillRect(0,0,c.width,c.height);
 const frame=data.frames[t];for(const [key,ps] of Object.entries(frame.regions))for(const [x,y] of ps){ctx.fillStyle=palette[key];ctx.fillRect(ox+x*sz,oy+(y-1)*sz,sz-2,sz-2);}
 ctx.strokeStyle='#142332';ctx.lineWidth=3;for(const [p,q] of frame.boundary_edges){const [x1,y1]=p,[x2,y2]=q;ctx.beginPath();if(x1!==x2){let x=ox+(Math.min(x1,x2)+1)*sz-1,y=oy+(y1-1)*sz;ctx.moveTo(x,y);ctx.lineTo(x,y+sz-2);}else{let x=ox+x1*sz,y=oy+Math.min(y1,y2)*sz-1;ctx.moveTo(x,y);ctx.lineTo(x+sz-2,y);}ctx.stroke();}
 for(const [k,a] of Object.entries(data.agents)){let [sx,sy]=pt(...a.start),[gx,gy]=pt(...a.goal);ctx.strokeStyle='#121e29';ctx.lineWidth=2;ctx.strokeRect(sx-9,sy-9,18,18);ctx.beginPath();ctx.moveTo(gx-8,gy-8);ctx.lineTo(gx+8,gy+8);ctx.moveTo(gx+8,gy-8);ctx.lineTo(gx-8,gy+8);ctx.stroke();let [x,y]=pt(...data.paths[k][t]);ctx.beginPath();ctx.arc(x,y,17,0,Math.PI*2);ctx.fillStyle=palette[k];ctx.fill();ctx.strokeStyle='#fff';ctx.lineWidth=3;ctx.stroke();ctx.fillStyle='#fff';ctx.font='bold 18px system-ui';ctx.textAlign='center';ctx.fillText(k,x,y+6);}
 ctx.font='bold 19px system-ui';ctx.fillStyle='#243141';ctx.textAlign='start';ctx.fillText('t = '+t+' / '+data.horizon+' | spatial cut edges: '+frame.boundary_count+' | switched cells: '+frame.temporal_switches,35,31);
 document.getElementById('step').textContent='  '+t+' / '+data.horizon;}
r.addEventListener('input',()=>draw(+r.value));draw(0);
</script></html>'''.replace('__DATA__',safe)
    out.write_text(content,encoding='utf8')


def compare_equal_ties(left:dict,right:dict,out:Path):
    names=list(COLORS)
    fig,axs=plt.subplots(1,2,figsize=(13.3,7.6),sharey=True,constrained_layout=True)
    for ax,r,title in ((axs[0],left,'Equal-cost solution 1: A / B first'),
                       (axs[1],right,'Equal-cost solution 2: C / D first')):
        arr=np.asarray([[names.index(ownership(fr)[(x,2)]) for x in range(3,12)]
                        for fr in r['multiway_cut']['frames']])
        H=r['horizon']
        ax.imshow(arr,cmap=ListedColormap([COLORS[k] for k in names]),vmin=-.5,vmax=3.5,
                  extent=(2.5,11.5,H+.5,-.5),aspect='auto',alpha=.78)
        for t in range(H+1):
            for j in range(8):
                if arr[t,j]!=arr[t,j+1]:ax.plot([j+3.5]*2,[t-.5,t+.5],c='#122030',lw=1.4)
        for n in names:
            p=r['schedule']['paths'][n]
            tx=[t for t,s in enumerate(p) if s[1]==2 and 3<=s[0]<=11]
            ax.plot([p[t][0] for t in tx],tx,color='white',lw=3)
            ax.plot([p[t][0] for t in tx],tx,color=COLORS[n],lw=1.4,ls='--')
        ax.set(xlabel='Bridge X',title=title,xticks=range(3,12),
               ylim=(H+.5,-.5),yticks=range(0,H+1,2))
        ax.text(.5,-.11,f'Sum = {r["schedule"]["sum_of_arrivals"]} steps  |  makespan = {r["schedule"]["makespan"]}',
                fontsize=10,ha='center',transform=ax.transAxes)
    axs[0].set_ylabel('Time step ↓')
    fig.suptitle('No prescribed priority: two symmetric best-found schedules',fontsize=15,weight='bold')
    fig.savefig(out,dpi=165);plt.close(fig)


def no_bay_four(out:Path):
    fig,ax=plt.subplots(figsize=(12,2.8))
    for x in range(15):
        ax.add_patch(Rectangle((x-.48,-.24),.96,.48,facecolor='#e6ebf0',edgecolor='white'))
    for k,x in [('A',0),('B',1),('C',13),('D',14)]:
        ax.scatter([x],[0],s=650,c=COLORS[k],edgecolor='white',linewidth=2,zorder=3)
        ax.text(x,0,k,ha='center',va='center',c='white',weight='bold',size=14,zorder=4)
    ax.annotate('',xy=(12,.55),xytext=(1,.55),arrowprops={'arrowstyle':'->','color':COLORS['A'],'lw':2.5})
    ax.annotate('',xy=(2,-.55),xytext=(13,-.55),arrowprops={'arrowstyle':'->','color':COLORS['C'],'lw':2.5})
    ax.set(xlim=(-.9,14.9),ylim=(-1.15,1.22),yticks=[],xticks=range(0,15,2),
           title='No side bays: 4 robots cannot reverse left–right order on a single line')
    ax.text(7,-.95,'No legal passing vertex / no head-on edge swap',ha='center',color='#943b39',fontsize=11)
    fig.tight_layout();fig.savefig(out,dpi=165);plt.close(fig)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--report',type=Path,default=Path('output/four_bridge/report.json'))
    p.add_argument('--figdir',type=Path,default=Path('docs/figures/four_bridge'))
    args=p.parse_args();args.figdir.mkdir(parents=True,exist_ok=True)
    report=json.loads(args.report.read_text(encoding='utf8'))
    if not report.get('feasible'):raise SystemExit('No feasible witness in this report')
    bridge_xt(report,args.figdir/'bridge_x_t_cut.png')
    slices(report,args.figdir/'bridge_four_time_slices.png')
    bridge_timeline(report,args.figdir/'bridge_passage_timeline.png')
    trajectory_3d(report,args.figdir/'four_directed_seed_paths.png')
    gif_bridge(report,args.figdir/'four_bridge_cut.gif')
    offline_explorer(report,args.figdir/'four_bridge_cut_slider.html')
    alt_path=args.report.with_name('right_first.json')
    if alt_path.is_file() and args.report.name!='right_first.json':
        alt=json.loads(alt_path.read_text(encoding='utf8'))
        compare_equal_ties(report,alt,args.figdir/'equal_cost_opposite_order.png')
    no_bay_four(args.figdir/'no_bays_four_impossible.png')
    print('Wrote figures, time-slice animation and offline slider:',args.figdir)

if __name__=='__main__':main()
