"""Small research prototype: D* Lite potentials + rolling XYT min-cut allocation.

This is deliberately a proof of concept, not a formally complete MAPF solver.
It uses NetworkX minimum_cut on a time-expanded spatial lattice to assign
exclusive ownership to two agents, then dynamic programming to search for
reachable paths within those ownership labels. Conflicts and reachability are
checked explicitly after the cut.

Run: python rolling_cut.py
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional

import networkx as nx

Point = tuple[int, int]
INF = float('inf')


@dataclass(frozen=True)
class Agent:
    name: str
    start: Point
    goal: Point
    priority: float = 1.0


class GridGraph:
    def __init__(self, cells: Iterable[Point]):
        self.cells = frozenset(cells)
        self.adj: dict[Point, tuple[Point, ...]] = {}
        for p in self.cells:
            x, y = p
            self.adj[p] = tuple(q for q in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)) if q in self.cells)
        if not self.cells:
            raise ValueError('empty grid')

    @classmethod
    def two_corridors(cls) -> 'GridGraph':
        """Short top corridor + longer lower bypass, both only one cell wide."""
        cells = {(x,2) for x in range(0,15)}
        cells.update((x,6) for x in range(1,14))
        cells.update((1,y) for y in range(2,7))
        cells.update((13,y) for y in range(2,7))
        cells.update({(0,3), (14,3)})
        return cls(cells)

    def neighbors(self, p: Point) -> tuple[Point, ...]:
        return self.adj[p]

    def distances(self, origin: Point) -> dict[Point, int]:
        d = {origin: 0}
        Q = deque([origin])
        while Q:
            p = Q.popleft()
            for q in self.neighbors(p):
                if q not in d:
                    d[q] = d[p]+1
                    Q.append(q)
        return d

    def route_avoiding(self, start:Point, goal:Point, avoid:set[Point]) -> list[Point]:
        """Unweighted shortest path excluding selected occupancy cells."""
        if start in avoid or goal in avoid: return []
        parents={start:None}
        Q=deque([start])
        while Q:
            p=Q.popleft()
            if p==goal:
                result=[]
                while p is not None:
                    result.append(p)
                    p=parents[p]
                return result[::-1]
            for q in self.neighbors(p):
                if q not in parents and q not in avoid:
                    parents[q]=p
                    Q.append(q)
        return []

    def pretty(self, a: Optional[Point]=None, b: Optional[Point]=None) -> str:
        xs = [p[0] for p in self.cells]; ys = [p[1] for p in self.cells]
        rows=[]
        for y in range(min(ys), max(ys)+1):
            s=''
            for x in range(min(xs), max(xs)+1):
                p=(x,y)
                s += 'X' if p == a == b else 'A' if p==a else 'B' if p==b else '.' if p in self.cells else '#'
            rows.append(s)
        return '\n'.join(rows)


class DStarLite:
    """Incremental D* Lite with reversed goal-based g/rhs values.

    Full propagation is used for this tiny demo so cost-to-go estimates are
    available for all grid cells when defining min-cut unary costs.
    """
    def __init__(self, graph: GridGraph, start: Point, goal: Point):
        self.graph = graph
        self.start=start; self.goal=goal
        self.last=start; self.km=0.0
        self.blocked: set[Point] = set()
        self.g: dict[Point,float] = {p:INF for p in graph.cells}
        self.rhs: dict[Point,float] = {p:INF for p in graph.cells}
        self.rhs[goal]=0.0
        self.heap: list[tuple[float,float,int,Point]]=[]
        self.open: dict[Point,tuple[float,float]] = {}
        self.counter=0
        self._insert(goal)
        self.compute(full=True)

    @staticmethod
    def _h(a: Point, b: Point) -> int:
        return abs(a[0]-b[0])+abs(a[1]-b[1])

    def _cost(self,u:Point,v:Point) -> float:
        return INF if u in self.blocked or v in self.blocked else 1.0

    def _key(self, u:Point) -> tuple[float,float]:
        k = min(self.g[u],self.rhs[u])
        return (k+self._h(self.start,u)+self.km, k)

    def _insert(self,u:Point) -> None:
        key=self._key(u)
        self.open[u]=key
        self.counter+=1
        heapq.heappush(self.heap, (*key,self.counter,u))

    def _clean(self) -> None:
        while self.heap:
            a,b,_,u=self.heap[0]
            if self.open.get(u)!=(a,b):
                heapq.heappop(self.heap)
            else:
                break

    def _pop(self) -> tuple[Point,tuple[float,float]]:
        self._clean()
        a,b,_,u=heapq.heappop(self.heap)
        del self.open[u]
        return u,(a,b)

    def update_vertex(self,u:Point) -> None:
        if u != self.goal:
            self.rhs[u] = min((self._cost(u,v)+self.g[v] for v in self.graph.neighbors(u)), default=INF)
        self.open.pop(u,None)
        if self.g[u] != self.rhs[u]:
            self._insert(u)

    def compute(self,full:bool=False) -> None:
        iterations=0
        while True:
            self._clean()
            top=(self.heap[0][0],self.heap[0][1]) if self.heap else (INF,INF)
            if not self.heap: break
            if not full and not (top < self._key(self.start) or self.rhs[self.start]!=self.g[self.start]):
                break
            iterations+=1
            if iterations > 100000:
                raise RuntimeError('D* Lite did not converge')
            u,old=self._pop()
            now=self._key(u)
            if old < now:
                self._insert(u)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for p in self.graph.neighbors(u): self.update_vertex(p)
            else:
                self.g[u]=INF
                self.update_vertex(u)
                for p in self.graph.neighbors(u): self.update_vertex(p)

    def move_start(self,p:Point) -> None:
        self.km += self._h(self.last,p)
        self.start=p; self.last=p
        self.compute(full=True)

    def change_obstacle(self,p:Point, blocked:bool) -> None:
        if p in (self.start,self.goal):
            raise ValueError('Cannot block start/goal')
        if blocked: self.blocked.add(p)
        else: self.blocked.discard(p)
        self.update_vertex(p)
        for v in self.graph.neighbors(p): self.update_vertex(v)
        self.compute(full=True)

    def path(self,start:Optional[Point]=None) -> list[Point]:
        p=self.start if start is None else start
        if p in self.blocked or math.isinf(self.g[p]): return []
        out=[p]
        for _ in range(len(self.graph.cells)+1):
            if p == self.goal: return out
            options=[v for v in self.graph.neighbors(p) if v not in out and v not in self.blocked]
            if not options: return []
            v=min(options,key=lambda v: (1.0+self.g[v],v))
            if math.isinf(self.g[v]): return []
            out.append(v); p=v
        return []


def min_cut_partition(
    graph:GridGraph, agents:list[Agent], positions:dict[str,Point],
    potentials:dict[str,DStarLite], horizon:int, clock:int,
    previous:dict[tuple[Point,int],str],
    protected_corridor:set[Point],
    *, path_protection:float=9., pairwise:float=.45, time_smooth:float=.6,
    flip_penalty:float=.12, escape_strength:float=0.0,
) -> tuple[dict[tuple[Point,int],str],float,dict]:
    """Binary Potts energy minimized exactly via s/t min cut on XYT cells.

    Agent A's priority-protected predicted corridor is a *soft* reservation.
    The actual motion planner later verifies reachability and collision safety.
    """
    a,b=agents
    src='SOURCE_A'; snk='SINK_B'
    flow=nx.DiGraph()
    flow.add_nodes_from([src,snk])
    A=a.name; B=b.name
    # Static detected conflict resource for this toy map, preserved across
    # windows even after the lower-priority robot has changed routes.
    # Recomputing it from the current best paths would drop ownership too soon.
    shared_corridor = protected_corridor
    lock_active = bool(shared_corridor) and positions[A] != a.goal
    # Candidate escape route: not itself the final B path; it only supplies
    # reachability-aware evidence to the cut. The time-expanded planner is free
    # to choose waiting, a bypass or any route left inside the cut region.
    escape=set(graph.route_avoiding(positions[B],b.goal,shared_corridor)) if lock_active and escape_strength>0 else set()
    dstart={k:graph.distances(positions[k]) for k in (A,B)}
    node_for={(p,t):(p[0],p[1],t) for p in graph.cells for t in range(horizon+1)}
    for (p,t),n in node_for.items():
        # Unary: approximate corridor preference based on D* cost-to-go and
        # cost-from-start. Strongest preferences come from deliberate priority.
        ac = .045*(dstart[A].get(p,99)+potentials[A].g[p])
        bc = .045*(dstart[B].get(p,99)+potentials[B].g[p])
        if lock_active and p in shared_corridor:
            bc += path_protection*a.priority
        if lock_active and p in escape:
            ac += escape_strength*b.priority
        # Last result retained very weakly to avoid excessive relabeling.
        label=previous.get((p,clock+t))
        if label == A: bc+=flip_penalty
        if label == B: ac+=flip_penalty
        # Force present positions to their owners.
        if t==0 and p==positions[A]: bc+=1e7
        if t==0 and p==positions[B]: ac+=1e7
        if p==a.goal: bc+=1e7
        if p==b.goal: ac+=1e7
        flow.add_edge(src,n,capacity=bc+1e-6)
        flow.add_edge(n,snk,capacity=ac+1e-6)

    # Undirected neighborhood Potts penalties as symmetric directed arcs.
    # This is the spatial/temporal graph-cut 'surface-area' term.
    for (p,t),n in node_for.items():
        for q in graph.neighbors(p):
            if p<q:
                m=node_for[q,t]
                flow.add_edge(n,m,capacity=pairwise)
                flow.add_edge(m,n,capacity=pairwise)
        if t<horizon:
            m=node_for[p,t+1]
            flow.add_edge(n,m,capacity=time_smooth)
            flow.add_edge(m,n,capacity=time_smooth)
    cut_value,(src_side,_) = nx.minimum_cut(flow,src,snk,capacity='capacity')
    labels = {(p,clock+t):A if n in src_side else B for (p,t),n in node_for.items()}
    info={'cut_value':cut_value,'corridor_locked':lock_active,
          'shared_shortest_path_cells':len(shared_corridor),
          'escape_candidate_cells':len(escape),
          'escape_strength':escape_strength,
          'label_A_count':sum(x==A for x in labels.values()),
          'label_B_count':sum(x==B for x in labels.values())}
    return labels,cut_value,info


def plan_within_partition(graph:GridGraph, agent:Agent, pos:Point, dstar:DStarLite,
                          labels:dict[tuple[Point,int],str], clock:int, horizon:int,
                          *, wait_cost:float=.22) -> Optional[list[Point]]:
    """Directed time-expanded (x,y,t) search under min-cut ownership.

    Since each motion advances time by one step, DP is an exact shortest-path
    calculation for this finite DAG. D* g is used as terminal cost-to-go.
    """
    if labels.get((pos,clock))!=agent.name:
        return None
    states:dict[Point,tuple[float,list[Point]]] = {pos:(0.0,[pos])}
    for t in range(1,horizon+1):
        nxt:dict[Point,tuple[float,list[Point]]]={}
        for p,(cost,path) in states.items():
            for q in (p,*graph.neighbors(p)):
                if labels.get((q,clock+t))!=agent.name: continue
                if q in dstar.blocked: continue
                step = 0.0 if p == q == agent.goal else 1.0
                extra = (wait_cost if q==p and p!=agent.goal else 0.0)
                # Discourage needless oscillations at a rolling horizon edge.
                if dstar.g[q] > dstar.g[p]: extra += 0.12
                c = cost + step + extra
                if q not in nxt or c < nxt[q][0]:
                    nxt[q]=(c,path+[q])
        states=nxt
        if not states: return None
    # Terminal potential drives progress to distant goal even in 10s windows.
    _,best = min(states.items(),key=lambda it:(it[1][0]+dstar.g[it[0]],
                                             dstar.g[it[0]],it[0]))
    return best[1]


def path_conflicts(paths:dict[str,list[Point]], names:list[str]) -> list[str]:
    a,b=(paths[k] for k in names)
    problems=[]
    for t in range(min(len(a),len(b))):
        if a[t]==b[t]: problems.append(f'vertex collision at t={t}, {a[t]}')
        if t and a[t-1]==b[t] and b[t-1]==a[t]:
            problems.append(f'edge swap at t={t}, {a[t-1]} <-> {a[t]}')
    return problems


def run(steps:int=20,horizon:int=10,execute:int=2,verbose:bool=True)->dict:
    graph=GridGraph.two_corridors()
    agents=[Agent('A',(0,2),(14,2),1.0), Agent('B',(14,3),(0,3),.85)]
    if horizon<2 or execute<1 or execute>horizon:
        raise ValueError('require horizon>=2 and 1<=execute<=horizon')
    positions={a.name:a.start for a in agents}
    dstar={a.name:DStarLite(graph,a.start,a.goal) for a in agents}
    baseline={a.name:dstar[a.name].path() for a in agents}
    global_traces={a.name:[a.start] for a in agents}
    shared = (set(baseline['A']) & set(baseline['B']))
    protected_corridor = set(shared)
    phases=[]; previous={}
    if verbose:
        print('GRID (A and B = starting positions)\n'+graph.pretty(positions['A'],positions['B']))
        print('INDEPENDENT D* SHORTEST PATHS:')
        for a in agents:
            p=baseline[a.name]
            print(f'  {a.name}: length={len(p)-1}; uses upper corridor: {any(y==2 and 1<x<13 for x,y in p)}')
    clock=0
    while clock<steps:
        remaining=min(execute,steps-clock)
        # Graph cut alone need not preserve directed reachability. First cut
        # without a bypass seed; then if a robot is disconnected, feed an
        # alternate-route witness back as additional unary costs and re-cut.
        attempts=0
        for escape_strength in (0.0, 9.0, 18.0):
            attempts+=1
            labels, cut, info=min_cut_partition(
                graph,agents,positions,dstar,horizon,clock,previous,
                protected_corridor,escape_strength=escape_strength)
            paths={a.name:plan_within_partition(
                graph,a,positions[a.name],dstar[a.name],labels,clock,horizon)
                   for a in agents}
            if all(p is not None for p in paths.values()): break
        if any(p is None for p in paths.values()):
            raise RuntimeError(f'cut disconnected a robot at t={clock} even after reachability feedback')
        info['cut_attempts']=attempts
        errors=path_conflicts(paths,[a.name for a in agents])
        if errors: raise RuntimeError(f'unsafe cut at t={clock}: {errors}')
        # Confirm that all executed vertices remain in their owners' regions.
        for a in agents:
            for dt,p in enumerate(paths[a.name]):
                assert labels[(p,clock+dt)]==a.name
        snapshots={
            str(offset): {
                name: [list(p) for p in sorted(graph.cells)
                       if labels[(p,clock+offset)]==name]
                for name in (agents[0].name,agents[1].name)
            }
            for offset in sorted({0,horizon//2,horizon})
        }
        rec={'time':clock,'cut_cost':round(cut,3),'info':info,
             'position_before':dict(positions),'planned_paths':paths,
             'partition_snapshots': snapshots,
             'executed':{a.name:paths[a.name][1:remaining+1] for a in agents}}
        phases.append(rec)
        if verbose:
            print(f'CUT t={clock:2d}..{clock+horizon}: '+
                  f'locked={info["corridor_locked"]}  '+
                  f'next {remaining}s A={rec["executed"]["A"]} B={rec["executed"]["B"]}')
        for a in agents:
            global_traces[a.name].extend(rec['executed'][a.name])
            positions[a.name]=paths[a.name][remaining]
            dstar[a.name].move_start(positions[a.name])
        # Retain *future* allocations as a weak hysteresis prior. Only the
        # executed prefix is immutable; future ownership may change at re-cut.
        previous={key:val for key,val in labels.items() if key[1]>=clock+remaining}
        clock+=remaining
        if all(positions[a.name]==a.goal for a in agents): break

    errors=path_conflicts(global_traces,[a.name for a in agents])
    if errors: raise RuntimeError('execution collisions: '+str(errors))
    result={
       'scenario':'two opposing UAVs; short single-width upper corridor and longer lower bypass',
       'parameters':{'horizon_s':horizon,'execute_s':execute,'max_simulated_s':steps},
       'grid':{'cells':[list(p) for p in sorted(graph.cells)]},
       'agents':{a.name:{'start':list(a.start),'goal':list(a.goal),
                         'independent_shortest_path':[list(p) for p in baseline[a.name]],
                         'trace':[list(p) for p in global_traces[a.name]],
                         'arrived':positions[a.name]==a.goal,
                         'arrival_time_s':next((i for i,p in enumerate(global_traces[a.name]) if p==a.goal),None)}
                 for a in agents},
       'phases':phases,
       'collisions':errors,
    }
    if verbose:
        print('RESULT:')
        for name,dat in result['agents'].items():
            print(f'  {name}: arrived={dat["arrived"]} arrival_t={dat["arrival_time_s"]} trace={dat["trace"]}')
        print('  collisions:',errors)
    return result


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--horizon',type=int,default=10)
    ap.add_argument('--execute',type=int,default=2)
    ap.add_argument('--steps',type=int,default=26)
    ap.add_argument('--output',type=Path,default=Path('output/demo.json'))
    a=ap.parse_args()
    result=run(steps=a.steps,horizon=a.horizon,execute=a.execute)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print('Saved:',a.output)

if __name__=='__main__': main()
