"""Four-agent, one-lane bridge: time-expanded routing and 4-label cut.

This is a discrete proof of concept, NOT a collision-avoidance guarantee.

Unlike binary source/sink min-cut, a 4-way Potts labeling is approximated by
alpha-expansion (one binary s/t min-cut per candidate label). We first compute
collision-free paths through an enumerated-order prioritized time-expanded A*
heuristic; those path cells are hard seeds in the labeling. Thus the graph cut
DOES NOT independently discover/guarantee directed connectivity or safety.

Run: python experiments/four_agent_bridge.py
"""
from __future__ import annotations

import argparse
import heapq
import itertools
import json
import math
import sys
from collections import deque
from pathlib import Path
from typing import Optional

import networkx as nx

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0,str(ROOT))
from rolling_cut import Agent, DStarLite, GridGraph

Point=tuple[int,int]
LabelNode=tuple[Point,int]
INF=10**12


def make_four_bridge(with_bays: bool=True) -> tuple[GridGraph,list[Agent]]:
    """Two robots on each bank, opposite destinations; exactly ONE center lane.

    Banks have staging cells; the 9-node central lane (3..11,y=2) has none.
    Goals differ from opponent starts so a robot can wait safely at its bank.
    """
    if with_bays:
        cells={(x,y) for x in range(0,3) for y in range(1,4)}
        cells.update((x,2) for x in range(3,12))
        cells.update((x,y) for x in range(12,15) for y in range(1,4))
        agents=[Agent('A',(0,1),(12,1)),Agent('B',(0,3),(12,3)),
                Agent('C',(14,1),(2,1)),Agent('D',(14,3),(2,3))]
    else:
        cells={(x,2) for x in range(15)}
        # A pair at each end on a pure path; reversing their order is impossible.
        agents=[Agent('A',(0,2),(12,2)),Agent('B',(1,2),(13,2)),
                Agent('C',(13,2),(1,2)),Agent('D',(14,2),(0,2))]
    return GridGraph(cells),agents


def independent_dstar(graph:GridGraph, agents:list[Agent]) -> tuple[dict[str,DStarLite],dict[str,list[Point]]]:
    dstar={a.name:DStarLite(graph,a.start,a.goal) for a in agents}
    paths={a.name:dstar[a.name].path() for a in agents}
    return dstar,paths


def pad_path(path:list[Point],H:int)->list[Point]:
    return path+[path[-1]]*(H+1-len(path))


def conflict_details(paths:dict[str,list[Point]])->list[dict]:
    """Vertex and opposing-edge swap conflicts at integer time steps."""
    if not paths: return []
    H=max(map(len,paths.values()))-1
    names=sorted(paths)
    pp={k:pad_path(p,H) for k,p in paths.items()}
    conflicts=[]
    for a,b in itertools.combinations(names,2):
        for t in range(H+1):
            if pp[a][t]==pp[b][t]:
                conflicts.append(dict(agents=[a,b],kind='vertex',step=t,point=list(pp[a][t])))
            if t and pp[a][t-1]==pp[b][t] and pp[b][t-1]==pp[a][t]:
                conflicts.append(dict(agents=[a,b],kind='edge_swap',step=t,
                                      segment=[list(pp[a][t-1]),list(pp[a][t])]))
    return conflicts


def reserved_astar(graph:GridGraph, agent:Agent, potential:DStarLite, H:int,
                   prior:dict[str,list[Point]])->Optional[list[Point]]:
    """Find earliest arrival avoiding all previously routed robots.

    Robots at Goal remain occupying their goal cell for the whole horizon.
    Earlier planned robots must not be forced to move by later ones.
    """
    busy={t:set() for t in range(H+1)}
    swaps={t:set() for t in range(1,H+1)}
    for path in prior.values():
        pp=pad_path(path,H)
        for t,p in enumerate(pp):
            busy[t].add(p)
            if t:
                swaps[t].add((pp[t],pp[t-1]))  # reversing this move is banned
    if agent.start in busy[0]:
        return None
    q=[]; cnt=itertools.count()
    st=(agent.start,0)
    heapq.heappush(q,(potential.g[agent.start],0,next(cnt),st))
    parents={st:None}
    cost={st:0}
    while q:
        _,g,_,(p,t)=heapq.heappop(q)
        if g!=cost.get((p,t)):continue
        if p==agent.goal:
            if all(p not in busy[tt] for tt in range(t+1,H+1)):
                rev=[];s=(p,t)
                while s is not None:
                    rev.append(s[0]);s=parents[s]
                return rev[::-1]
        if t>=H:continue
        for r in (p,*sorted(graph.neighbors(p))):
            # On a one-cell-wide bridge, do not enter and then reverse back
            # to the same bank. Wait outside until uninterrupted crossing is
            # feasible. This is a motion rule, NOT an agent-priority rule.
            if 3 <= p[0] <= 11 and p[1] == 2:
                forward = 1 if agent.goal[0] > agent.start[0] else -1
                if (r[0]-p[0])*forward < 0:
                    continue
            if r in busy[t+1] or (p,r) in swaps[t+1]:
                continue
            if t+1+potential.g[r]>H:
                continue
            new=(r,t+1)
            if t+1 >= cost.get(new,INF): continue
            cost[new]=t+1;parents[new]=(p,t)
            heapq.heappush(q,(t+1+potential.g[r],t+1,next(cnt),new))
    return None


def route_by_enumerated_orders(graph:GridGraph,agents:list[Agent],
                              dstar:dict[str,DStarLite],H:int,
                              tie_side:Optional[str]=None)->dict:
    """Enumerate all 4! planning orders; minimize sum arrival, then makespan.

    This is an order-enumeration PRIORITIZED ROUTING HEURISTIC, not globally
    optimal four-agent MAPF, and not a 4-label graph-cut trajectory solver.
    No permanent robot priority is chosen upfront; best feasible order wins.
    """
    if tie_side not in (None,'left','right'):
        raise ValueError('tie_side must be left, right, or None')
    solutions=[]
    tested=0
    for order in itertools.permutations([a.name for a in agents]):
        tested+=1
        paths={}
        for name in order:
            a=next(a for a in agents if a.name==name)
            path=reserved_astar(graph,a,dstar[name],H,paths)
            if path is None: break
            paths[name]=path
        if len(paths)!=len(agents):continue
        errs=conflict_details(paths)
        assert not errs,errs
        arrival={k:len(v)-1 for k,v in paths.items()}
        score=(sum(arrival.values()),max(arrival.values()))
        solutions.append((score,order,paths,arrival))
    if not solutions:
        return {'found':False,'tested_orders':tested,'feasible_orders':0,
                'explanation':'No schedule found by enumerated-order prioritized A* within horizon; not a proof of infeasibility.'}
    # Tie preference only breaks EXACT equal primary sum-of-arrivals/makespan.
    # It never overrides a lower-cost schedule or reserves the bridge.
    def choice_key(v):
        order=v[1]
        side_is_favored=(order[0] in ('A','B') if tie_side=='left'
                         else order[0] in ('C','D') if tie_side=='right'
                         else True)
        return (v[0],not side_is_favored,order)
    solutions.sort(key=choice_key)
    (soc,makespan),order,paths,arrival=solutions[0]
    return {'found':True,'tested_orders':tested,'feasible_orders':len(solutions),
            'planning_order':list(order),'tie_side':tie_side or 'none',
            'arrival_steps':arrival,
            'sum_of_arrivals':soc,'makespan':makespan,
            'collision_checks':[],
            'paths':{k:[list(p) for p in pad_path(v,H)] for k,v in sorted(paths.items())},
            'top_candidates':[{'sum_of_arrivals':s[0][0],'makespan':s[0][1],
                               'planning_order':list(s[1])} for s in solutions[:10]],
            'method':'enumeration of 24 prioritized space-time A* planning orders (heuristic, no fixed priority)'}


def unary_costs(graph:GridGraph, agents:list[Agent],potentials:dict[str,DStarLite],H:int,
                required:Optional[dict[str,list[Point]]]=None,
                *,hard:int=100_000_000)->tuple[list[LabelNode],dict[LabelNode,dict[str,int]],dict[LabelNode,str]]:
    starts={a.name:graph.distances(a.start) for a in agents}
    names=[a.name for a in agents]
    nodes=[(p,t) for t in range(H+1) for p in sorted(graph.cells)]
    pinned={}
    for a in agents:
        pinned[(a.start,0)]=a.name
        pinned[(a.goal,H)]=a.name
    if required:
        for k,path in required.items():
            assert len(path)==H+1
            for t,p in enumerate(path):
                if (p,t) in pinned and pinned[(p,t)]!=k:
                    raise ValueError('conflicting witness pins')
                pinned[(p,t)]=k
    unary={}
    for p,t in nodes:
        vals={}
        for a in agents:
            ds=starts[a.name][p]
            dg=potentials[a.name].g[p]
            # Smooth costs guide areas to agents reachable FROM their start
            # and TO their goal within the planning horizon; not hard guards.
            temporal_violation=max(0,ds-t)+max(0,dg-(H-t))
            val=int(round(100*(0.06*(ds+dg)+0.36*temporal_violation)))
            if (p,t) in pinned and pinned[(p,t)]!=a.name:val+=hard
            vals[a.name]=val
        unary[(p,t)]=vals
    return nodes,unary,pinned


def neighbor_edges(graph:GridGraph,H:int,spatial_lambda:float,temporal_lambda:float):
    """Weighted adjacency of the CUT graph, not valid robot motion edges."""
    ws=int(round(spatial_lambda*100));wt=int(round(temporal_lambda*100))
    for t in range(H+1):
        for p in sorted(graph.cells):
            for q in graph.neighbors(p):
                if p<q:yield ((p,t),(q,t),ws)
            if t<H:yield ((p,t),(p,t+1),wt)


def compute_energy(labels:dict[LabelNode,str],unary:dict[LabelNode,dict[str,int]],edges)->int:
    return (sum(unary[n][lab] for n,lab in labels.items())+
            sum(w for u,v,w in edges if labels[u]!=labels[v]))


def alpha_move(labels:dict[LabelNode,str],alpha:str,
               unary:dict[LabelNode,dict[str,int]],edges,
               *,pinned:dict[LabelNode,str]) -> dict[LabelNode,str]:
    """Exact binary alpha-expansion move using an integer-capacity min cut.

    x=0 keeps old label, x=1 switches to alpha.
    Pairwise metric Potts E00/E01/E10/E11 is converted into unary shifts
    plus K*|x_u-x_v| where K=(E01+E10-E00-E11)/2 >=0.
    All capacities are doubled to avoid half-integers.
    """
    src='__SRC__';snk='__SNK__'
    graph=nx.DiGraph();graph.add_nodes_from([src,snk]);graph.add_nodes_from(unary)
    keep={n:2*data[labels[n]] for n,data in unary.items()}
    switch={n:2*data[alpha] for n,data in unary.items()}
    for u,v,w in edges:
        lu,lv=labels[u],labels[v]
        e00=2*w*(lu!=lv);e01=2*w*(lu!=alpha)
        e10=2*w*(alpha!=lv);e11=0
        k=(e01+e10-e00-e11)//2
        assert 2*k==e01+e10-e00-e11 and k>=0
        au=e10-e00-k; bv=e01-e00-k
        if au>=0:switch[u]+=au
        else:keep[u]+=-au
        if bv>=0:switch[v]+=bv
        else:keep[v]+=-bv
        if k:
            graph.add_edge(u,v,capacity=k)
            graph.add_edge(v,u,capacity=k)
    forced_cost=10**12
    for n,lab in pinned.items():
        if lab==alpha:keep[n]+=forced_cost
        else:switch[n]+=forced_cost
    for n in unary:
        graph.add_edge(src,n,capacity=int(switch[n]))
        graph.add_edge(n,snk,capacity=int(keep[n]))
    cut,(side,_)=nx.minimum_cut(graph,src,snk,capacity='capacity',
                                flow_func=nx.algorithms.flow.preflow_push)
    updated={n:labels[n] if n in side else alpha for n in unary}
    assert all(updated[n]==lab for n,lab in pinned.items())
    return updated


def four_way_cut(graph:GridGraph,agents:list[Agent], dstar:dict[str,DStarLite],H:int,
                 required:Optional[dict[str,list[Point]]], *,passes:int=3,
                 spatial_lambda:float=0.45,temporal_lambda:float=1.2)->dict:
    """Four-way metric-Potts approximate multiway cut with hard route seeds."""
    nodes,unary,pinned=unary_costs(graph,agents,dstar,H,required)
    edges=list(neighbor_edges(graph,H,spatial_lambda,temporal_lambda))
    names=sorted(a.name for a in agents)
    labels={n:min(names,key=lambda name:(unary[n][name],name)) for n in nodes}
    labels.update(pinned)
    initial_energy=compute_energy(labels,unary,edges)
    hist=[initial_energy]
    for _ in range(passes):
        updated=False
        for name in names:
            candidate=alpha_move(labels,name,unary,edges,pinned=pinned)
            e=compute_energy(candidate,unary,edges)
            if e< hist[-1]:
                labels=candidate;hist.append(e);updated=True
        if not updated:break
    assert all(labels[k]==v for k,v in pinned.items())
    return {'labels':labels,'statistics':{'algorithm':'integer-capacity Potts alpha-expansion / multi-label cut (approximate)',
            'nodes':len(nodes),'spatiotemporal_edges':len(edges),
            'energy_initial':initial_energy,'energy_final':hist[-1],
            'energy_history':hist,'seeded_witness_paths':required is not None,
            'hard_seed_nodes':len(pinned),'spatial_lambda':spatial_lambda,
            'temporal_lambda':temporal_lambda}}


def region_paths(graph:GridGraph, agents:list[Agent],labels:dict[LabelNode,str],H:int,
                 witness:dict[str,list[Point]])->tuple[dict[str,list[Point]],dict[str,bool]]:
    """Within each label, independently compute shortest *directed* path.

    Replan requires explicit collision check afterward: exclusive XYT vertices
    do not by themselves prevent opposite-direction edge swaps.
    """
    paths={};reachable={}
    for a in agents:
        # Dynamic programming: minimize first arrival time while remaining at
        # the goal through the FINAL horizon layer. This ensures the path is
        # allocated at every step, including after its first goal arrival.
        parent={(a.start,0):None}
        states={a.start:0}
        for t in range(H):
            nxt={}
            for p,arrival_cost in states.items():
                for r in ((p,) if p==a.goal else (p,*graph.neighbors(p))):
                    if 3 <= p[0] <= 11 and p[1] == 2:
                        forward = 1 if a.goal[0] > a.start[0] else -1
                        if (r[0]-p[0])*forward < 0:continue
                    if labels.get((r,t+1)) != a.name:continue
                    c=arrival_cost+(0 if p==a.goal else 1)
                    if r not in nxt or c<nxt[r]:
                        nxt[r]=c
                        parent[(r,t+1)]=(p,t)
            states=nxt
        if a.goal in states:
            seq=[];cur=(a.goal,H)
            while cur is not None:
                seq.append(cur[0]);cur=parent[cur]
            paths[a.name]=seq[::-1]
            reachable[a.name]=True
        else:reachable[a.name]=False
    return paths,reachable


def four_label_frames(graph:GridGraph,labels:dict[LabelNode,str],names:list[str],H:int)->list[dict]:
    frames=[]
    for t in range(H+1):
        frame={'t':t,'regions':{n:[] for n in names},'boundary_edges':[],
               'boundary_count':0,'temporal_switches':0}
        for p in sorted(graph.cells):
            frame['regions'][labels[(p,t)]].append(list(p))
            for q in graph.neighbors(p):
                if p<q and labels[(p,t)]!=labels[(q,t)]:
                    frame['boundary_edges'].append([list(p),list(q)])
            if t and labels[(p,t)]!=labels[(p,t-1)]:frame['temporal_switches']+=1
        frame['boundary_count']=len(frame['boundary_edges'])
        frames.append(frame)
    return frames


def run_four(*,factor:float=2.,cut_passes:int=3,with_bays:bool=True,
             tie_side:Optional[str]=None)->dict:
    graph,agents=make_four_bridge(with_bays)
    dstar,independent=independent_dstar(graph,agents)
    H=math.ceil(factor*max(len(p)-1 for p in independent.values()))
    common={'scenario':'four_bridge_with_bays' if with_bays else 'four_bridge_no_bays',
            'horizon':H,'horizon_factor':factor,'space_time_nodes':len(graph.cells)*(H+1),
            'grid':[list(p) for p in sorted(graph.cells)],
            'bridge':[list((x,2)) for x in range(3,12)] if with_bays else [list((x,2)) for x in range(15)],
            'agents':{a.name:{'start':list(a.start),'goal':list(a.goal),
                      'independent_steps':len(independent[a.name])-1,
                      'independent_route':[list(p) for p in independent[a.name]]} for a in agents},
            'independent_path_conflicts':conflict_details(independent)}
    if not with_bays:
        common['feasible']=False
        common['proof']='On a path graph with vertex and opposing-edge exclusion, left-to-right robot order is invariant. Opposite-end exchange is impossible without side branches.'
        return common
    schedule=route_by_enumerated_orders(graph,agents,dstar,H,tie_side=tie_side)
    common['schedule']=schedule
    if not schedule['found']:
        common['feasible']=False
        return common
    common['feasible']=True
    required={name:[tuple(p) for p in seq] for name,seq in schedule['paths'].items()}
    four=four_way_cut(graph,agents,dstar,H,required,passes=cut_passes)
    labels=four['labels']
    paths,reachable=region_paths(graph,agents,labels,H,required)
    errors=conflict_details(paths) if len(paths)==len(agents) else []
    witness_errors=conflict_details(required)
    assert not witness_errors
    assert all(reachable.values())
    pure=four_way_cut(graph,agents,dstar,H,None,passes=cut_passes)
    _,pure_reachable=region_paths(graph,agents,pure['labels'],H,{})
    common['unseeded_cut']={'statistics':pure['statistics'],'start_goal_directed_reachable':pure_reachable,
        'frames':four_label_frames(graph,pure['labels'],[a.name for a in agents],H)}
    common['multiway_cut']={
        'statistics':four['statistics'],
        'frames':four_label_frames(graph,labels,[a.name for a in agents],H),
        'start_goal_directed_reachable':reachable,
        'replanned_arrival_steps':{k:next(t for t,p in enumerate(seq) if p==next(a.goal for a in agents if a.name==k)) for k,seq in paths.items()},
        'replanned_path_conflicts':errors,
        'replanned_paths':{k:[list(p) for p in pad_path(seq,H)] for k,seq in paths.items()},
        'guarantee':'Pinned witness paths are directed-connected and vertex/edge-swap collision-free on discrete grid; independent replan is checked but not automatically safe.',
    }
    return common


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--factor',type=float,default=2.)
    p.add_argument('--passes',type=int,default=3)
    p.add_argument('--tie-side',choices=['left','right'],default=None,
                   help='Tie-break ONLY equal-cost schedules, not hard priority')
    p.add_argument('--out',type=Path,default=Path('output/four_bridge/report.json'))
    a=p.parse_args()
    out=run_four(factor=a.factor,cut_passes=a.passes,tie_side=a.tie_side)
    out['no_bays']=run_four(factor=a.factor,with_bays=False)
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf8')
    print('Horizon:',out['horizon'],'nodes:',out['space_time_nodes'])
    print('Independent:',{k:v['independent_steps'] for k,v in out['agents'].items()})
    print('Schedule:',{k:v for k,v in out['schedule'].items() if k!='paths' and k!='top_candidates'})
    if 'multiway_cut' in out:
        print('Cut:',out['multiway_cut']['statistics'])
        print('Replanned:',out['multiway_cut']['replanned_arrival_steps'])
        print('Replanned conflicts:',out['multiway_cut']['replanned_path_conflicts'])
    print('No-bays:',out['no_bays']['proof'])
    print('Saved:',a.out)

if __name__=='__main__':main()
