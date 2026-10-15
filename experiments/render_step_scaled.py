"""Render a four-panel PNG of the step-scaled/no-priority experiment.

Uses Pillow already present in the repository requirements.
Run from repo root: python experiments/render_step_scaled.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from rolling_cut import GridGraph  # noqa: E402

BACKGROUND = (247, 249, 252)
PANEL = (255, 255, 255)
BORDER = (210, 217, 227)
FREE = (224, 230, 235)
WALL = (70, 83, 98)
A = (54, 114, 190)
B = (219, 133, 54)
INK = (35, 49, 66)
MUTED = (93, 104, 117)


def font(size: int, bold: bool = False):
    p = '/usr/share/fonts/truetype/dejavu/DejaVuSans' + ('-Bold.ttf' if bold else '.ttf')
    try:
        return ImageFont.truetype(p, size)
    except OSError:
        return ImageFont.load_default()


def write_figure(data: dict, filename: Path):
    width, height = 1720, 850
    im = Image.new('RGB', (width, height), BACKGROUND)
    d = ImageDraw.Draw(im)
    d.text((38, 25), 'XYT Graph Cut: 32-step full-horizon experiment', font=font(32, True), fill=INK)
    d.text((39, 72), 'Independent D* Lite  ->  symmetric cut  ->  reachability feedback  ->  A* inside cut', font=font(17), fill=MUTED)
    d.rounded_rectangle((1370, 32, 1668, 81), radius=10, fill=(232, 240, 249))
    d.text((1395, 44), 'No preset agent priority', font=font(15, True), fill=A)

    locations = [(34, 121), (874, 121), (34, 464), (874, 464)]
    titles = [
        '1. Independent shortest routes',
        '2. Pure symmetric cut, t = 16',
        '3. Repaired partition, t = 16',
        '4. Routes recomputed within cut',
    ]
    subtitles = [
        'Both agents initially choose the upper corridor',
        'A can arrive, but B is disconnected',
        'Connected witnesses were fed back into graph cut',
        'A upper: 14 steps    |    B lower: 20 steps',
    ]
    graph = GridGraph.two_corridors()
    free = graph.cells
    independent = data['independent']
    repair = data['reachability_feedback']['selected']
    pure = data['pure_symmetric_cut']['slices']['16']
    adjusted = data['reachability_feedback']['selected_cut_slices']['16']
    scale = 36

    for idx, ((ox, oy), title, subtitle) in enumerate(zip(locations, titles, subtitles)):
        right, bottom = ox + 811, oy + 325
        d.rounded_rectangle((ox, oy, right, bottom), radius=15, fill=PANEL, outline=BORDER, width=2)
        d.text((ox+24, oy+22), title, font=font(21, True), fill=INK)
        d.text((ox+24, oy+55), subtitle, font=font(15), fill=MUTED)
        zx = ox + 128
        zy = oy + 100
        map_a = {tuple(p) for p in adjusted['A']}
        map_b = {tuple(p) for p in adjusted['B']}
        pure_a = {tuple(p) for p in pure['A']}
        pure_b = {tuple(p) for p in pure['B']}
        for y in range(2, 7):
            for x in range(15):
                pos = (x, y)
                x0,y0=zx+x*scale,zy+(y-2)*scale
                if pos not in free:
                    continue
                color=FREE
                if idx==1: color=A if pos in pure_a else B if pos in pure_b else FREE
                if idx==2: color=A if pos in map_a else B if pos in map_b else FREE
                d.rounded_rectangle((x0+1, y0+1, x0+scale-2, y0+scale-2),radius=5,fill=color)
                if idx in (1,2):
                    d.rectangle((x0+1, y0+1, x0+scale-2, y0+scale-2),outline=(255,255,255),width=1)

        def route_points(path):
            return [(zx+int(p[0])*scale+scale//2, zy+(int(p[1])-2)*scale+scale//2) for p in path]
        def paint_path(path, color, side=0):
            pts=route_points(path)
            if side:
                pts=[(x, y+side) for x,y in pts]
            if len(pts)>1:
                d.line(pts,fill=color,width=7,joint='curve')
            for p in (pts[0],pts[-1]):
                d.ellipse((p[0]-7,p[1]-7,p[0]+7,p[1]+7),fill=color,outline=PANEL,width=2)
        if idx==0:
            paint_path(independent['A']['path'],A,-7)
            paint_path(independent['B']['path'],B,7)
        if idx==3 and repair is not None:
            for name,color in [('A',A),('B',B)]:
                end=repair['arrival_steps'][name]
                paint_path(repair['replanned_paths'][name][:end+1],color)
        d.text((ox+24,oy+286), 'A (blue)    B (orange)    free cell (grey)', font=font(16), fill=MUTED)

    d.text((42, 807), 'Min-cut alone disconnects B; symmetric reachability feedback is required. Toy grid, no continuous-time safety proof.', font=font(16), fill=MUTED)
    filename.parent.mkdir(parents=True, exist_ok=True)
    im.save(filename)
    return filename


def main():
    source=ROOT/'output/step_scaled_no_priority.json'
    if not source.exists():
        raise RuntimeError('Run python experiments/step_scaled_horizon.py first')
    out=ROOT/'output/step_scaled_comparison.png'
    write_figure(json.loads(source.read_text(encoding='utf-8')),out)
    print('Saved',out)

if __name__=='__main__':
    main()
