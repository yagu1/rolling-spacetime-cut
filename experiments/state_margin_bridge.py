"""Four-agent bridge: state-dependent XYT potentials, geometric margins, seeded vs unseeded multiway cut.

Research prototype. Cut labels alone do NOT certify directed reachability or continuous collision safety.
"""
from __future__ import annotations
import argparse, json, math, sys
from pathlib import Path
import networkx as nx
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from experiments.four_agent_bridge import (make_four_bridge,independent_dstar,unary_costs,neighbor_edges,compute_energy,alpha_move,region_paths,four_label_frames,route_by_enumerated_orders,pad_path)


def time_potentials(graph,agents,dstar,H,mode='state'):
    """Full XYT unary energy with start-arrival wavefront, goal-reachability and time-evolving preferred progress.
    These are soft labels, NOT path seeds. Each node gets a finite score for all agents.
    """
    starts={a.name:graph.distances(a.start) for a in agents}
    unary={}; fixed={}
    for t in range(H+1):
        for p in sorted(graph.cells):
            key=(p,t)
            unary[key]={}
            for a in agents:
                f=starts[a.name][p]; g=dstar[a.name].g[p]; shortest=starts[a.name][a.goal]
                detour=f+g-shortest
                if mode=='static':
                    val=.06*(f+g)+.36*(max(0,f-t)+max(0,g-(H-t)))
                else:
                    # Arrival-state potential: wavefront follows progress through time.
                    # Penalize spatial locations inconsistent with plausible clock state.
                    target=min(shortest,max(0,t - .16 * max(0,H-shortest)))
                    progression=abs(f-target)
                    early=max(0,f-t)
                    late=max(0,t+g-H)
                    val=.15*detour+.26*progression+.75*(early+late)
                    # Slight preference to place recently visited cells behind moving front.
                    val+=.05*abs(g-max(0,shortest-t))
                unary[key][a.name]=int(round(100*val))
    for a in agents:
        fixed[(a.start,0)]=a.name
        fixed[(a.goal,H)]=a.name
    return unary,fixed


def cut_labels(graph,agents,dstar,H,mode='state',passes=2):
    unary,pinned=time_potentials(graph,agents,dstar,H,mode)
    edges=list(neighbor_edges(graph,H,spatial_lambda=.45,temporal_lambda=1.4))
    names=[a.name for a in agents]
    labels={n:min(names,key=lambda z:(u[z],z)) for n,u in unary.items()}
    labels.update(pinned)
    energies=[compute_energy(labels,unary,edges)]
    for _ in range(passes):
        changed=False
        for a in names:
            cand=alpha_move(labels,a,unary,edges,pinned=pinned)
            cost=compute_energy(cand,unary,edges)
            if cost<energies[-1]:
                labels=cand;energies.append(cost);changed=True
        if not changed:break
    _,reachable=region_paths(graph,agents,labels,H,{})
    return {'labels':labels,'reachable':reachable,'energies':energies,
            'frames':four_label_frames(graph,labels,names,H)}


def dist_to_segment_zero(a,b,c,d):
    # Synchronously moving points over one unit time, minimizing relative separation.
    ux=a[0]-c[0]; uy=a[1]-c[1]
    vx=(b[0]-a[0])-(d[0]-c[0]); vy=(b[1]-a[1])-(d[1]-c[1])
    vv=vx*vx+vy*vy
    t=max(0,min(1,-(ux*vx+uy*vy)/vv)) if vv else 0
    return math.hypot(ux+t*vx,uy+t*vy)


def min_clearance(paths):
    names=sorted(paths)
    length=min(len(paths[n]) for n in names)
    best=float('inf'); pair=None; tbest=None
    for i,a in enumerate(names):
        for b in names[i+1:]:
            for t in range(1,length):
                d=dist_to_segment_zero(paths[a][t-1],paths[a][t],paths[b][t-1],paths[b][t])
                if d<best:best=d;pair=(a,b);tbest=t
    return {'distance':best,'pair':pair,'transition':tbest}


def padded_clearance(paths,margin):
    return min_clearance(paths)['distance']+1e-9 >= margin


def run():
    graph,agents=make_four_bridge(True)
    dstar,independent=independent_dstar(graph,agents)
    H=2*max(len(p)-1 for p in independent.values())
    records={'horizon':H,'cell_count':len(graph.cells),'agents':[a.name for a in agents],
             'assumptions':['1 grid step = 1 meter; 1 time step = 1 second',
                            'Robot geometry approximated by circular horizontal footprints',
                            'Clearance on paths checked over linearly interpolated unit-time transitions',
                            'Environment walls represented by grid connectivity; geometric obstacle clearance not established'],
             'unseeded_cuts':{},'witness_comparison':{}}
    for mode in ['static','state']:
        cut=cut_labels(graph,agents,dstar,H,mode)
        records['unseeded_cuts'][mode]={
            'reachable':cut['reachable'],'energy_history':cut['energies'],
            'frames':cut['frames']}
    # The prior version's candidate is used only as a diagnostic feasible path witness,
    # NOT as input to the unseeded cuts above.
    schedule=route_by_enumerated_orders(graph,agents,dstar,H)
    assert schedule['found']
    paths={name:[tuple(p) for p in coords] for name,coords in schedule['paths'].items()}
    closest=min_clearance(paths)
    records['witness_comparison']={'arrival':schedule['arrival_steps'],
        'planning_order':schedule['planning_order'],'minimum_continuous_pair_distance_m':closest,
        'checks':{str(m):{'separation_required_m':round(m,3),'paths_pass':padded_clearance(paths,m)} for m in [.0,.6,.9,1.1,1.4]},
        'paths':schedule['paths']}
    records["margin_aware_planning"]=margin_schedules(graph,agents,dstar,H,[0.0,0.6,0.9,1.1,1.4])
    return records


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',default='output/state_margin/report.json');arg=p.parse_args()
    result=run();dest=ROOT/arg.out;dest.parent.mkdir(exist_ok=True,parents=True)
    dest.write_text(json.dumps(result,indent=2),encoding='utf8')
    print('Horizon',result['horizon'],'grid cells',result['cell_count'])
    for k,v in result['unseeded_cuts'].items():print(k,'directed connectivity',v['reachable'],'energy',v['energy_history'][-1])
    print('Path witness',result['witness_comparison']['arrival'])
    print('Minimum continuous pairwise distance',result['witness_comparison']['minimum_continuous_pair_distance_m'])
    print('Margin checks',result['witness_comparison']['checks'])
    for s,d in result['margin_aware_planning'].items():print('separation',s,'found',d['found'],'arrival',d.get('arrival'),'minimum',d.get('minimum_distance_m'))
    print('Saved',dest)


def clearance_astar(graph, agent, potential, H, prior, minimum_distance):
    """Prioritized A* enforcing pairwise swept center-distance against reserved paths."""
    import heapq, itertools
    if any(math.dist(agent.start, path[0]) < minimum_distance - 1e-9 for path in prior.values()):return None
    seqcount=itertools.count(); heap=[(potential.g[agent.start],0,next(seqcount),agent.start,0)]
    parent={(agent.start,0):None};seen={(agent.start,0):0}
    while heap:
        _,g,_,p,t=heapq.heappop(heap)
        if g!=seen.get((p,t)):continue
        if p==agent.goal:
            if all(all(math.dist(p, pp[tt])>=minimum_distance-1e-9 for tt in range(t,H+1)) for pp in prior.values()):
                seq=[];cur=(p,t)
                while cur is not None:seq.append(cur[0]);cur=parent[cur]
                return pad_path(seq[::-1],H)
        if t>=H:continue
        for q in (p,*sorted(graph.neighbors(p))):
            if 3<=p[0]<=11 and p[1]==2:
                forward=1 if agent.goal[0]>agent.start[0] else -1
                if (q[0]-p[0])*forward<0:continue
            if any(dist_to_segment_zero(p,q,pp[t],pp[t+1])<minimum_distance-1e-9 for pp in prior.values()):continue
            if t+1+potential.g[q]>H:continue
            state=(q,t+1)
            if state in seen:continue
            seen[state]=t+1;parent[state]=(p,t)
            heapq.heappush(heap,(t+1+potential.g[q],t+1,next(seqcount),q,t+1))
    return None


def margin_schedules(graph,agents,dstar,H,separations):
    import itertools
    result={}
    byname={a.name:a for a in agents}
    for sep in separations:
        feasible=[]
        for order in itertools.permutations(byname):
            prior={}
            for name in order:
                path=clearance_astar(graph,byname[name],dstar[name],H,prior,sep)
                if path is None:break
                prior[name]=path
            if len(prior)==len(agents):
                arrival={k:next(t for t,p in enumerate(v) if p==byname[k].goal) for k,v in prior.items()}
                closest=min_clearance(prior)
                if closest['distance']+1e-9 < sep:continue
                feasible.append((sum(arrival.values()),max(arrival.values()),order,prior,arrival,closest))
        feasible.sort(key=lambda x:(x[0],x[1],x[2]))
        if feasible:
            soc,makespan,order,paths,arrival,closest=feasible[0]
            result[str(sep)]={'found':True,'feasible_orders':len(feasible),'sum_of_arrivals':soc,
             'makespan':makespan,'planning_order':list(order),'arrival':arrival,
             'minimum_distance_m':closest,'paths':{k:[list(p) for p in v] for k,v in paths.items()}}
        else:result[str(sep)]={'found':False,'feasible_orders':0,
                              'note':'No solution found by 24-order prioritized A*; does NOT prove global infeasibility'}
    return result

if __name__=="__main__": main()
