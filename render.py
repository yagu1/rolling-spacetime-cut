"""Render a static comparison, an initial cut snapshot, and an animated GIF.

No code execution in the viewer required: all images are pre-rendered.
Usage: python render.py --input output/demo.json --output-dir output
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

CELL=42
LEFT=82
TOP=76
COLORS={'A':'#2774be','B':'#d78332'}
BG='#f9fafb'
FLOOR='#e4e8ec'
WALL='#bcc4ca'
INK='#263442'


def font(size=18,bold=False):
    path='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf' if bold else '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    try: return ImageFont.truetype(path,size)
    except OSError: return ImageFont.load_default()


def base(result,header,footer='2D+T discrete proof-of-concept; not a continuous safety guarantee'):
    im=Image.new('RGB',(800,410),BG)
    d=ImageDraw.Draw(im)
    d.text((24,18),header,font=font(24,True),fill=INK)
    cells={tuple(p) for p in result['grid']['cells']}
    for y in range(2,7):
        for x in range(0,15):
            xa=LEFT+x*CELL; ya=TOP+(y-2)*CELL
            d.rounded_rectangle((xa,ya,xa+CELL-3,ya+CELL-3),radius=4,
                                fill=FLOOR if (x,y) in cells else WALL)
    for a in ('A','B'):
        goal=tuple(result['agents'][a]['goal'])
        px=LEFT+goal[0]*CELL+CELL/2-1;py=TOP+(goal[1]-2)*CELL+CELL/2-1
        d.rectangle((px-9,py-9,px+9,py+9),outline=COLORS[a],width=3)
    d.text((30,301),'A = priority agent  |  B = lower-priority agent',font=font(16),fill=INK)
    d.text((30,334),footer,font=font(13),fill='#576777')
    d.text((30,358),'Grid time step = 1 s; planning horizon = 10 s; execution step = 2 s',font=font(13),fill='#576777')
    return im


def xy(p): return (LEFT+p[0]*CELL+CELL/2-1,TOP+(p[1]-2)*CELL+CELL/2-1)


def line(d,points,color,width=6):
    if len(points)>1: d.line([xy(tuple(p)) for p in points],fill=color,width=width,joint='curve')


def dot(d,p,color):
    x,y=xy(tuple(p));d.ellipse((x-11,y-11,x+11,y+11),fill=color,outline='white',width=2)


def render_baseline(result,path):
    im=base(result,'Independent D* Lite paths: both prefer the upper corridor')
    d=ImageDraw.Draw(im)
    for k in ('A','B'):
        data=result['agents'][k]
        line(d,data['independent_shortest_path'],COLORS[k],5)
        dot(d,data['start'],COLORS[k])
    im.save(path)


def render_final(result,path):
    A=result['agents']['A'];B=result['agents']['B']
    im=base(result,f'After rolling graph cut: A {A["arrival_time_s"]} s, B {B["arrival_time_s"]} s')
    d=ImageDraw.Draw(im)
    for k in ('A','B'):
        data=result['agents'][k]
        line(d,data['trace'],COLORS[k],5)
        dot(d,data['start'],COLORS[k])
    im.save(path)


def render_cut(result,path):
    phase=result['phases'][0]
    image=Image.new('RGB',(840,500),BG)
    draw=ImageDraw.Draw(image)
    draw.text((22,15),'Initial space-time graph cut (three time slices)',fill=INK,font=font(23,True))
    xorig=66
    for index,(offset,labels) in enumerate(phase['partition_snapshots'].items()):
        yorig=77+index*132
        draw.text((22,yorig),f't={offset}s',font=font(15,True),fill=INK)
        owner={tuple(p):k for k,cells in labels.items() for p in cells}
        walk={tuple(p) for p in result['grid']['cells']}
        for y in range(2,7):
            for x in range(15):
                if (x,y) not in walk:continue
                bx=xorig+x*CELL;by=yorig+(y-2)*22
                c=owner[(x,y)]
                draw.rectangle((bx,by,bx+CELL-3,by+18),fill=COLORS[c],outline='white',width=1)
        draw.text((712,yorig),f'A: {len(labels["A"])}',font=font(13),fill=COLORS['A'])
        draw.text((712,yorig+26),f'B: {len(labels["B"])}',font=font(13),fill=COLORS['B'])
    draw.text((24,469),'Colored blocks indicate ownership; reachability is checked after the cut.',font=font(13),fill='#576777')
    image.save(path)


def render_animation(result,path):
    a=result['agents']['A']['trace'];b=result['agents']['B']['trace']
    end=max(len(a),len(b))-1
    frames=[]
    for t in range(end+1):
        im=base(result,f'Rolling XYT allocation — simulated t = {t:02d} s')
        d=ImageDraw.Draw(im)
        for name,trace in (('A',a),('B',b)):
            prefix=trace[:min(t+1,len(trace))]
            line(d,prefix,COLORS[name],5)
            dot(d,prefix[-1],COLORS[name])
        d.text((30,386),f'A: {a[min(t,len(a)-1)]}   B: {b[min(t,len(b)-1)]}',font=font(13),fill=INK)
        frames.append(im)
    frames[0].save(path,save_all=True,append_images=frames[1:],duration=450,loop=0,optimize=True)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--input',type=Path,default=Path('output/demo.json'))
    ap.add_argument('--output-dir',type=Path,default=Path('output'))
    args=ap.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    data=json.loads(args.input.read_text(encoding='utf-8'))
    render_baseline(data,args.output_dir/'independent_routes.png')
    render_final(data,args.output_dir/'allocated_routes.png')
    render_cut(data,args.output_dir/'cut_snapshots.png')
    render_animation(data,args.output_dir/'simulation.gif')
    for p in sorted(args.output_dir.iterdir()): print(f'CREATED {p}')

if __name__=='__main__': main()
