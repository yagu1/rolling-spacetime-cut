"""Reproduce the four fixed-horizon / interrobot-margin comparisons.

From repo root: python experiments/run_reachability_margin_matrix.py
"""
from __future__ import annotations
import csv
import json
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from experiments.reachability_margin_cut import run

CONDITIONS=((28,.60),(28,.90),(30,.90),(30,1.10))

def main():
    records=[]
    out=ROOT/'output'/'reachability_margin';out.mkdir(parents=True,exist_ok=True)
    for H,dreq in CONDITIONS:
        r=run(H=H,body_radius=.28,wall_margin=.1,pair_margin=dreq-.56)
        final=r['safety_feedback_cut']
        joint=final['joint_restricted'] if final else {}
        geom=final.get('geometry') if final else None
        row={
            'horizon_steps':H,'pair_required_center_m':round(dreq,2),
            'initial_connected_agents':sum(r['initial_cut']['connected'].values()),
            'directed_feedback_connected_agents':sum(r['directed_feedback_cut']['connected'].values()),
            'oracle_found':r['safety_oracle']['found'],
            'final_joint_plan_found':bool(joint.get('found',False)),
            'arrival_A':joint.get('arrival',{}).get('A'),
            'arrival_B':joint.get('arrival',{}).get('B'),
            'arrival_C':joint.get('arrival',{}).get('C'),
            'arrival_D':joint.get('arrival',{}).get('D'),
            'minimum_swept_pair_center_m':round(geom['minimum_pair_center_m'],5) if geom else None,
            'minimum_swept_wall_center_m':round(geom['minimum_wall_center_m'],5) if geom else None,
        }
        records.append(row)
        print(row,flush=True)
    (out/'matrix.json').write_text(json.dumps(records,indent=2),encoding='utf8')
    with (out/'matrix.csv').open('w',newline='',encoding='utf8') as f:
        writer=csv.DictWriter(f,fieldnames=list(records[0]))
        writer.writeheader();writer.writerows(records)
    print('Saved',out/'matrix.json',out/'matrix.csv')

if __name__=='__main__':main()
