"""Monte Carlo allocation budget study; deterministic map, multiple random seeds.

Time measurements are local hardware observations, NOT universal runtime bounds.
"""
from __future__ import annotations
import csv
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from experiments.hierarchical_restriction import run

def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out=ROOT/'output/hierarchical_restriction'
    out.mkdir(parents=True,exist_ok=True)
    budgets=[2,8,16,32,64]
    seeds=list(range(10))
    rows=[]
    for budget in budgets:
        for seed in seeds:
            r=run(budget=budget,seed=seed)
            rows.append({'budget':budget,'seed':seed,'all_arrived':int(r['completed']),
                         'sum_arrival_seconds':sum(r['arrivals'].values()),
                         'makespan_seconds':max(r['arrivals'].values()),
                         'min_center_distance_m':r['minimum_swept_distance_m'],
                         'total_supervisor_compute_ms':1000*sum(u['search_seconds'] for u in r['supervisor_updates']),
                         'update_count':len(r['supervisor_updates'])})
    with (out/'budget_study.csv').open('w',newline='',encoding='utf8') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    avg=lambda b,k:sum(r[k] for r in rows if r['budget']==b)/len(seeds)
    means=[avg(b,'sum_arrival_seconds') for b in budgets]
    compute=[avg(b,'total_supervisor_compute_ms') for b in budgets]
    success=[sum(r['all_arrived'] for r in rows if r['budget']==b) for b in budgets]
    fig,axes=plt.subplots(1,2,figsize=(10,3.8),layout='constrained')
    axes[0].plot(budgets,means,'-o')
    axes[0].set_xlabel('Sampled allocation candidates / update')
    axes[0].set_ylabel('Mean sum of arrival times (s)')
    axes[0].set_title('Random shooting search quality (10 seeds)')
    axes[1].plot(budgets,compute,'-o')
    axes[1].set_xlabel('Sampled allocation candidates / update')
    axes[1].set_ylabel('Mean total supervisor compute time (ms)')
    axes[1].set_title('Compute budget on this machine')
    for ax in axes:ax.grid(alpha=.2)
    fig.savefig(out/'budget_study.png',dpi=170,bbox_inches='tight')
    plt.close(fig)
    print('budget mean_arrival_sum mean_supervisor_compute_ms completed_runs')
    for b,m,c,s in zip(budgets,means,compute,success):
        print(b,round(m,2),round(c,2),f'{s}/{len(seeds)}')

if __name__=='__main__':main()
