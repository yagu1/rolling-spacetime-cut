"""Reproducible contingency sweep: stochastic supervisor, fixed toy scenarios.

Do not treat these tests as statistically representative of real environments.
Each seed changes only the randomized restriction candidate generation. Agent
positions, map geometry, obstacle timings and idealized safety monitor are fixed.
"""
from __future__ import annotations
import argparse
import csv
import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from experiments.contingency_restriction import run_case


def evaluate(seeds=range(10),budget=16,max_time=90):
    rows=[]
    for scenario in ('none','approach_detour','bridge_closure'):
        for seed in seeds:
            for method in ('restriction','central_path','uncoordinated'):
                r=run_case(scenario,method,seed=seed,budget=budget,max_time=max_time)
                rows.append({
                    'scenario':scenario,'seed':seed,'method':method,
                    'completed':r['all_arrived'],
                    'completion_time':r['completion_time'],
                    'sum_arrival_times':r['sum_arrival_times'],
                    'emergency_stops':r['emergency_stops'],
                    'minimum_swept_distance':r['minimum_swept_distance'],
                    'local_replans':sum(r['planner_calls'].values()),
                    'path_messages':r['path_dispatch_messages'],
                    'restriction_messages':r['restriction_messages'],
                    'path_payload_bytes':r['path_payload_bytes'],
                    'restriction_payload_bytes':r['restriction_payload_bytes'],
                    'runtime_seconds':r['runtime_seconds'],
                })
    grouped=defaultdict(list)
    for r in rows:grouped[(r['scenario'],r['method'])].append(r)
    summary=[]
    for (scenario,method),gr in sorted(grouped.items()):
        successful=[r['completion_time'] for r in gr if r['completed']]
        summary.append({
            'scenario':scenario,'method':method,'trials':len(gr),
            'successes':len(successful),'success_rate':len(successful)/len(gr),
            'mean_completion_s_successful_only':
                statistics.mean(successful) if successful else None,
            'std_completion_s_successful_only':
                statistics.stdev(successful) if len(successful)>1 else (0 if successful else None),
            'mean_stops':statistics.mean(r['emergency_stops'] for r in gr),
            'mean_compute_runtime_s':statistics.mean(r['runtime_seconds'] for r in gr),
        })
    return rows,summary


def write_csv(path:Path,rows:list[dict]):
    with path.open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader()
        writer.writerows(rows)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--seeds',type=int,default=10)
    p.add_argument('--budget',type=int,default=16)
    p.add_argument('--max-time',type=int,default=90)
    p.add_argument('--out',default='output/contingency_restriction')
    args=p.parse_args()
    rows,summary=evaluate(range(args.seeds),args.budget,args.max_time)
    out=ROOT/args.out;out.mkdir(parents=True,exist_ok=True)
    write_csv(out/'seed_sweep.csv',rows)
    write_csv(out/'seed_sweep_summary.csv',summary)
    (out/'seed_sweep.json').write_text(json.dumps({'trials':rows,'summary':summary},indent=2),encoding='utf-8')
    for x in summary:print(x)
    print('Saved',out)

if __name__=='__main__':main()
