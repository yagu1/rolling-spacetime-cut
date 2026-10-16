"""Dynamic-obstacle experiments for restriction-only multi-rover navigation.

New experiment (not the established full-XYT Graph Cut):
  * Fixed delivery Start/Goal assignment; task reassignment not implemented.
  * Shared, randomized gate planning for a single-lane bridge every 10 s.
  * Local onboard obstacle sensing, incremental D* Lite, temporal A*, or safe hold.
  * Compare restriction-only navigation, 10-s fixed-path dispatch, and no gate.

The safety arbiter is an explicitly idealized, common simulation safeguard,
NOT a proof of decentralized collision avoidance. It uses ground-truth states
and rejects simulated simultaneous transitions violating swept-center margins.
This is an exploratory deterministic toy-grid model, not suitable for field use.
"""
from __future__ import annotations

import argparse
import csv
import heapq
import itertools
import json
import math
import sys
import time
from dataclasses import dataclass, asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments.hierarchical_restriction import (
    AccessWindow, RestrictionMap, Rover, Supervisor, swept_clearance,
)
from experiments.four_agent_bridge import make_four_bridge
from rolling_cut import GridGraph
from experiments.state_margin_bridge import dist_to_segment_zero

Point = tuple[int,int]


@dataclass(frozen=True)
class TemporaryObstacle:
    cell: Point
    active_from: int
    active_until: int
    name: str

    def active(self, t: int) -> bool:
        return self.active_from <= t < self.active_until


SCENARIOS = {
    # Sudden right-bank obstacle on an initially useful approach to the bridge;
    # bank-side connectors allow a rover to discover a local bypass.
    'approach_detour': (TemporaryObstacle((13, 2), 1, 18, 'staging-area obstruction'),),
    # Direct temporary closure of the only shared bridge; robots cannot detour
    # and must hold until the obstruction clears.
    'bridge_closure': (TemporaryObstacle((7, 2), 6, 23, 'bridge passage obstruction'),),
    'none': (),
}


class ContingencyRover(Rover):
    """Independent navigation: onboard local observations, map and restriction.

    The supervisor never injects a route into this class. Time-dependent A* is
    rerun by the rover on each local control tick, even between RP updates.
    """
    def __init__(self, agent, graph, *, radius=.28, wall_margin=.10,
                 sensor_radius:int=2):
        super().__init__(agent,graph,radius=radius,wall_margin=wall_margin)
        self.sensor_radius = sensor_radius
        self.known_obstacles: set[Point] = set()
        self.new_observations: list[dict] = []
        self.local_replans = 0
        self._in_rollout = False
        self.obstacle_replans = 0
        self.safe_hold_count = 0
        self.emergency_stops = 0
        self.local_detours = 0
        self.last_plan_status = 'unplanned'
        self.nominal_g = float(self.dstar.g[self.pos])
        self._seen_change_at = None

    def scan(self, actual_blocked:set[Point], now:int) -> bool:
        """Senses occupancy of nearby cells; remembers unobserved obstacles.

        A previously seen blocked cell is released ONLY after observing it
        cleared inside sensor range. There is no omniscient global update.
        """
        changed = False
        for p in sorted(self.graph.cells):
            if abs(p[0]-self.pos[0])+abs(p[1]-self.pos[1]) > self.sensor_radius:
                continue
            before = p in self.known_obstacles
            after = p in actual_blocked
            if before == after or p == self.pos or p == self.agent.goal:
                continue
            if after:
                self.known_obstacles.add(p)
            else:
                self.known_obstacles.discard(p)
            self.dstar.change_obstacle(p,after)
            self.new_observations.append({'time':now,'cell':list(p),'blocked':after})
            changed = True
        if changed:
            self.obstacle_replans += 1
            self._seen_change_at = now
        return changed

    def local_plan(self, position:Point, now:int, horizon:int,
                   restriction:RestrictionMap) -> list[Point]:
        """Rover-internal temporal A*; permits waiting and no fixed trajectory.

        When the goal is temporarily unreachable, returns a verified *hold*
        candidate rather than falsely claiming that a route exists. This does
        not certify that the hold will remain safe under all external actions.
        """
        if not self._in_rollout:
            self.local_replans += 1
        if position == self.agent.goal:
            self.last_plan_status='completed'
            return [position]*(horizon+1)
        if not restriction.permits(position,now):
            # An already-occupied bridge cell cannot be instantly revoked;
            # if the supervisor violates this invariant, fail clearly.
            raise ValueError(f'active cell {position} revoked by restriction')

        # Do not use a separate path from the supervisor. D* supplies the only
        # goal potential. Blocked visible obstacles update D* incrementally.
        if not math.isfinite(self.dstar.g[position]):
            self.last_plan_status='safe_hold_no_path'
            if not self._in_rollout:self.safe_hold_count += 1
            return [position]*(horizon+1)
        q=[]; counter=itertools.count()
        first=(position,now)
        heapq.heappush(q,(self.dstar.g[position], 0, next(counter), first))
        best={first:0}; parent={first:None}; hit=None
        while q:
            _,g,_,state=heapq.heappop(q)
            if g != best[state]:continue
            p,t=state
            if p == self.agent.goal:
                hit=state;break
            if t >= now+horizon:continue
            # Wait is a valid, neutral action. Prior demo's added wait penalty
            # induced pointless back-and-forth motion while waiting for grants.
            for v in (p,*sorted(self.safe.get(p,()))):
                if v in self.known_obstacles:continue
                if not restriction.permits(v,t+1):continue
                if not math.isfinite(self.dstar.g[v]):continue
                # On a one-cell bridge, maintain direction after entering.
                if 3<=p[0]<=11 and p[1]==2 and v!=p:
                    forward=1 if self.agent.goal[0]>self.agent.start[0] else -1
                    if (v[0]-p[0])*forward<0:continue
                new=(v,t+1); ng=g+1
                if ng>=best.get(new,math.inf):continue
                best[new]=ng;parent[new]=state
                heapq.heappush(q,(ng+self.dstar.g[v],ng,next(counter),new))
        if hit is None:
            self.last_plan_status='safe_hold_no_grant'
            if not self._in_rollout:self.safe_hold_count += 1
            return [position]*(horizon+1)
        rev=[]
        while hit is not None:
            rev.append(hit[0]);hit=parent[hit]
        route=rev[::-1]
        self.last_plan_status='route'
        return route + [self.agent.goal]*(horizon+1-len(route))


class ResilientSupervisor(Supervisor):
    """Same finite-budget Monte Carlo resource grant search as v0.3.

    An obstruction may make a goal temporarily unreachable. Do not discard a
    candidate just because an independent rover can only safely hold for now.
    Temporal grants are revised at 10-second boundaries. In-progress access
    is frozen; the 60-second prediction is not a promise of future occupancy.
    """
    def _evaluate(self, now:int, windows:dict[str,AccessWindow],*,version:int):
        paths={};restrictions={}; arrivals={}
        holds=0
        for name,rover in self.rovers.items():
            mask=RestrictionMap(version,now,now+self.lookahead,self.resource,
                    windows.get(name), frozenset(r.agent.goal
                    for other,r in self.rovers.items() if other!=name))
            if not mask.permits(rover.pos,now):return None
            rover._in_rollout=True
            try:
                path=rover.local_plan(rover.pos,now,self.lookahead,mask)
            finally:
                rover._in_rollout=False
            paths[name]=path;restrictions[name]=mask
            if rover.agent.goal not in path:
                arrivals[name]=self.lookahead+10
                holds+=1
            else:
                arrivals[name]=path.index(rover.agent.goal)
        safe,distance,repulsion=swept_clearance(paths,self.required)
        if not safe:return None
        churn=sum(abs(w.open_at-self.windows[n].open_at)
                  for n,w in windows.items() if n in self.windows)
        # Safety is hard, positive hold penalty discourages unnecessary closure.
        restriction_volume = sum(sum(not (w.open_at <= t < w.close_at) for t in range(now,now+self.lookahead)) for w in windows.values())
        score=sum(arrivals.values())+2*holds+0.07*repulsion+0.025*churn+0.001*restriction_volume
        return {'score':score,'paths':paths,'restrictions':restrictions,
                'arrival_remaining':arrivals,'minimum_distance':distance,
                'repulsion':repulsion,'churn':churn,'windows':windows,'holds':holds,
                'restriction_volume':restriction_volume}

    def update(self,now:int):
        # Need to preserve occupancy if a temporary obstruction delays a rover
        # beyond its old gate. The old supervisor freezes a grant once open,
        # but here active bridge occupation may outlast a planned grant. Keep
        # access until the rover physically clears; extend before replanning.
        for name,rover in self.rovers.items():
            old=self.windows.get(name)
            if rover.pos in self.resource and old and old.close_at<now+self.lookahead:
                self.windows[name]=AccessWindow(old.open_at,now+self.lookahead)
        try:
            return super().update(now)
        except (RuntimeError,ValueError):
            # Conservative fallback: if no candidate is verified, do not
            # issue a newly unsafe grant. Existing occupied rover keeps its
            # resource; all uncommitted rovers receive no access.
            self.version += 1
            restrictions={}
            for n,r in self.rovers.items():
                active=r.pos in self.resource
                old=self.windows.get(n)
                win=(AccessWindow(min(old.open_at,now) if old else now,
                                  now+self.lookahead) if active else None)
                if active:self.windows[n]=win
                else:self.windows.pop(n,None)
                restrictions[n]=RestrictionMap(self.version,now,now+self.lookahead,
                    self.resource,win,
                    frozenset(q.agent.goal for k,q in self.rovers.items() if k!=n))
                r.accept_restriction(restrictions[n])
            self.history.append({'now':now,'version':self.version,
                'feasible_candidates':0,'fallback_safe_hold':True,
                'cut_calls':0,'scope':self._conflict_scope(now)})
            return {'restrictions':restrictions,'windows':dict(self.windows),
                    'score':None,'fallback_safe_hold':True}


def make_contingency_bridge():
    """Bridge with right-bank side connectors; no extra crossing lane.

    Branch lanes allow a robot to bypass a *local approach obstruction*
    without bypassing the shared bridge itself. Existing goal parking squares
    stay reserved, so the bypass loops around those parking squares.
    """
    graph,agents=make_four_bridge(with_bays=True)
    cells=set(graph.cells)
    cells.update((x,0) for x in range(11,15))
    cells.update((x,4) for x in range(11,15))
    cells.update({(11,1),(11,3)})
    return GridGraph(cells), agents


def current_obstacles(events:tuple[TemporaryObstacle,...],t:int)->set[Point]:
    return {event.cell for event in events if event.active(t)}


def no_gate(rover:ContingencyRover,now:int,H:int,res:set[Point],rovers):
    return RestrictionMap(-1,now,now+H,frozenset(res),AccessWindow(now,now+H),
        frozenset(r.agent.goal for name,r in rovers.items() if name!=rover.agent.name))


def shield_moves(rovers:dict[str,ContingencyRover], proposed:dict[str,Point],
                 required:float, *, actual_blocked:set[Point]) -> tuple[dict[str,Point],list[str]]:
    """Idealized one-step physical safety monitor shared across all baselines.

    Uses ground truth and all four positions; NOT onboard distributed logic.
    Compares all move/wait combinations, so a stop imposed on one robot is
    checked for conflicts with the other robots that continue moving.
    """
    names=sorted(rovers)
    legal=[]
    for bits in itertools.product((0,1),repeat=len(names)):
        nxt={name: (proposed[name] if bits[i] else rovers[name].pos)
             for i,name in enumerate(names)}
        if any(nxt[n] in actual_blocked for n in names):continue
        if any(nxt[n] not in rovers[n].safe.get(rovers[n].pos,()) for n in names):continue
        ok=True
        for a,b in itertools.combinations(names,2):
            d=dist_to_segment_zero(rovers[a].pos,nxt[a],rovers[b].pos,nxt[b])
            if d < required-1e-9:ok=False;break
        if ok:
            # Small reward for approaching the assigned goal, but a move must
            # not be forced if it only oscillates because of gate closure.
            progress=sum(max(0,rovers[n].dstar.g[rovers[n].pos]-
                         rovers[n].dstar.g[nxt[n]])
                         if math.isfinite(rovers[n].dstar.g[nxt[n]]) else 0
                         for n in names)
            moves=sum(nxt[n]!=rovers[n].pos for n in names)
            legal.append((progress,moves,bits,nxt))
    if not legal:
        raise RuntimeError('No safe joint 1-step transition, even all-stop')
    _,_,_,selected=max(legal,key=lambda r:(r[0],r[1],r[2]))
    stopped=[name for name in names if proposed[name]!=rovers[name].pos
             and selected[name]==rovers[name].pos]
    return selected,stopped


def run_case(scenario:str='approach_detour',mode:str='restriction',*,
             seed:int=17,budget:int=16,lookahead:int=60,
             update_every:int=10,max_time:int=90,
             safety_distance:float=.90,sensor_radius:int=2) -> dict:
    if scenario not in SCENARIOS:raise ValueError('invalid scenario')
    if mode not in ('restriction','central_path','uncoordinated'):
        raise ValueError('invalid mode')
    graph, agents=make_contingency_bridge()
    events=SCENARIOS[scenario]
    rovers={a.name:ContingencyRover(a,graph,radius=.28,wall_margin=.10,
                                   sensor_radius=sensor_radius) for a in agents}
    supervisor=ResilientSupervisor(rovers,lookahead=lookahead,update_every=update_every,
                  candidate_budget=budget,safety_distance=safety_distance,seed=seed)
    baseline_resource=set(supervisor.resource)
    # Central-path baseline uses the SAME coordinator/gate assignment, but
    # sends route commands at each 10-s dispatch boundary. It cannot locally
    # reroute between dispatches; it can stop on unexpected obstacles.
    central_plans:dict[str,list[Point]]={}
    central_plan_clock=0
    path_dispatch_count=0
    path_payload_bytes=0
    restriction_payload_bytes=0
    sim_start=time.perf_counter()
    timeline=[]; restricted_messages=[]
    updates=0; reactive_replans=0; emergency_stops=0; observations=0
    opposing_grant_ticks=0
    for now in range(max_time):
        actual=current_obstacles(events,now)
        newly={}
        for n,rover in rovers.items():
            newly[n]=rover.scan(actual,now)
            observations+=int(newly[n])
        if now % update_every == 0:
            if mode!='uncoordinated':
                s=supervisor.update(now)
                updates+=1
                restricted_messages.extend({'time':now,'agent':n,
                    'payload':r.restriction.payload()} for n,r in rovers.items())
                restriction_payload_bytes += sum(len(json.dumps(r.restriction.payload(),separators=(',',':')).encode('utf-8')) for r in rovers.values())
            if mode=='central_path':
                central_plan_clock=now
                central_plans={n:r.local_plan(r.pos,now,lookahead,r.restriction)
                               for n,r in rovers.items()}
                path_dispatch_count+=len(rovers)
                path_payload_bytes += sum(len(json.dumps({'issued_at':now,'waypoints':p},separators=(',',':')).encode('utf-8')) for p in central_plans.values())
        # RP should never simultaneously grant the one-lane shared resource
        # to traffic arriving from opposite sides. This is only a resource
        # grant invariant, not a general collision-avoidance guarantee.
        if mode!='uncoordinated':
            open_grants={n for n,w in supervisor.windows.items()
                if w.open_at<=now<w.close_at}
            if open_grants.intersection({'A','B'}) and open_grants.intersection({'C','D'}):
                opposing_grant_ticks += 1
        proposed={}
        states={}
        for n,r in rovers.items():
            if mode=='uncoordinated':
                mask=no_gate(r,now,lookahead,baseline_resource,rovers)
            else:
                mask=r.restriction
                assert mask is not None
            if mode=='central_path':
                target_index=now-central_plan_clock+1
                path=central_plans[n]
                proposed[n]=path[target_index] if target_index<len(path) else r.pos
                states[n]='central_path_command'
            else:
                path=r.local_plan(r.pos,now,lookahead,mask)
                proposed[n]=path[1]
                states[n]=r.last_plan_status
                if newly[n]:reactive_replans+=1
            if proposed[n] in actual:
                # Onboard immediate obstacle stop in all methods.
                proposed[n]=r.pos
                states[n]='obstacle_stop'
                r.emergency_stops+=1
                emergency_stops+=1
            if proposed[n] not in r.safe.get(r.pos,()):
                proposed[n]=r.pos
                states[n]='stale_route_stop'
                r.emergency_stops+=1
                emergency_stops+=1
        selected,shield_stops=shield_moves(rovers,proposed,safety_distance,
                                          actual_blocked=actual)
        emergency_stops+=len(shield_stops)
        for n in shield_stops:
            rovers[n].emergency_stops += 1
        prev={n:r.pos for n,r in rovers.items()}
        for n,r in rovers.items():
            if selected[n] in actual:
                raise AssertionError('ground truth obstacle entered')
            if selected[n]!=r.pos and mode!='uncoordinated':
                assert r.restriction.permits(selected[n],now+1)
            r.pos=selected[n]
            r.trace.append(r.pos)
            if r.arrived_at is None and r.pos == r.agent.goal:
                r.arrived_at=now+1
            r.dstar.move_start(r.pos)
        timeline.append({'time':now,'obstacles':[list(p) for p in sorted(actual)],
            'position_before':{n:list(p) for n,p in prev.items()},
            'positions':{n:list(r.pos) for n,r in rovers.items()},
            'status':states,'shield_stops':shield_stops,
            'observed_changed':newly,
            'grant_windows':{n:asdict(w) for n,w in supervisor.windows.items()}
                           if mode!='uncoordinated' else {}})
        if all(r.arrived_at is not None for r in rovers.values()):break
    completed=all(r.arrived_at is not None for r in rovers.values())
    el=time.perf_counter()-sim_start
    traces={n:r.trace for n,r in rovers.items()}
    passed,dmin,_=swept_clearance(traces,safety_distance)
    # Step-by-step safe occupancy and wall feasibility are checked separately.
    assert passed and dmin>=safety_distance-1e-9
    overlap_events=[(n,k,traces[n][k]) for n in traces for k in range(len(traces[n])) if traces[n][k] in current_obstacles(events,k)]
    assert not overlap_events, (scenario,mode,overlap_events[:10])
    return {
        'scenario':scenario,'method':mode,
        'notes':['fixed Start/Goal assignments; TA optimization not included',
                 '1 grid cell=1m; step=1s; robot radius=.28m',
                 'supervisor restriction-only if mode=restriction',
                 'upper bridge is single resource; no geometric Graph Cut',
                 'local obstacle observations within Manhattan sensor_radius',
                 'ground-truth joint transition safety shield is idealized and SHARED by ALL methods',
                 'swept pairwise separation checked, no realistic sensor/control/communication noise'],
        'settings':{'update_interval':update_every,'lookahead':lookahead,'budget':budget,
                    'safety_distance':safety_distance,'sensor_radius':sensor_radius,'seed':seed},
        'events':[{'name':e.name,'cell':list(e.cell),
                   'active_from':e.active_from,'active_until':e.active_until}
                  for e in events],
        'all_arrived':completed,'arrival_times':{n:r.arrived_at for n,r in rovers.items()},
        'completion_time':max((r.arrived_at for r in rovers.values() if r.arrived_at is not None),default=None)
                       if completed else None,
        'sum_arrival_times':sum(r.arrived_at or max_time for r in rovers.values()),
        'simulation_steps':len(timeline),'minimum_swept_distance':dmin,
        'safety_distance_satisfied':passed,'emergency_stops':emergency_stops,
        'opposing_direction_simultaneous_grant_ticks':opposing_grant_ticks,
        'sensing_update_count':observations,'reactive_local_replans':reactive_replans,
        'path_dispatch_messages':path_dispatch_count,
        'restriction_messages':len(restricted_messages),
        'path_payload_bytes':path_payload_bytes,
        'restriction_payload_bytes':restriction_payload_bytes,
        'restriction_payload_contains_paths':any('path' in k.lower() or 'waypoint' in k.lower()
            for z in restricted_messages for k in z['payload']),
        'planner_calls':{n:r.local_replans for n,r in rovers.items()},
        'safe_hold_plans':{n:r.safe_hold_count for n,r in rovers.items()},
        'obstacle_replans':{n:r.obstacle_replans for n,r in rovers.items()},
        'observations':{n:r.new_observations for n,r in rovers.items()},
        'traces':{n:[list(p) for p in r.trace] for n,r in rovers.items()},
        'supervisor_updates':supervisor.history if mode!='uncoordinated' else [],
        'timeline':timeline,'restriction_payloads':restricted_messages,
        'runtime_seconds':el,'cut_calls':0,
    }


def suite(seed:int=17,budget:int=16,lookahead:int=60,update_every:int=10,
          max_time:int=90)->list[dict]:
    rows=[]
    for scenario in ('none','approach_detour','bridge_closure'):
        for mode in ('restriction','central_path','uncoordinated'):
            rows.append(run_case(scenario,mode,seed=seed,budget=budget,
                lookahead=lookahead,update_every=update_every,max_time=max_time))
    return rows


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--seed',type=int,default=17)
    parser.add_argument('--budget',type=int,default=16)
    parser.add_argument('--lookahead',type=int,default=60)
    parser.add_argument('--update',type=int,default=10)
    parser.add_argument('--max-time',type=int,default=90)
    parser.add_argument('--out',default='output/contingency_restriction')
    opts=parser.parse_args()
    destination=ROOT/opts.out;destination.mkdir(parents=True,exist_ok=True)
    records=suite(opts.seed,opts.budget,opts.lookahead,opts.update,opts.max_time)
    for report in records:
        path=destination/f"{report['scenario']}_{report['method']}.json"
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    columns=['scenario','method','all_arrived','completion_time','sum_arrival_times',
             'minimum_swept_distance','emergency_stops','reactive_local_replans',
             'path_dispatch_messages','restriction_messages',
             'path_payload_bytes','restriction_payload_bytes',
             'simulation_steps','runtime_seconds']
    with (destination/'summary.csv').open('w',newline='',encoding='utf-8') as fh:
        w=csv.DictWriter(fh,fieldnames=columns);w.writeheader()
        for r in records:w.writerow({k:r.get(k) for k in columns})
    for r in records:
        print(f"{r['scenario']:18s} {r['method']:15s} "
              f"arrived={r['all_arrived']} makespan={str(r['completion_time']):>4s} "
              f"stop={r['emergency_stops']:>3d} min_sep={r['minimum_swept_distance']:.2f} "
              f"msgs={r['restriction_messages']}R/{r['path_dispatch_messages']}P "
              f"run={r['runtime_seconds']:.2f}s")
    print('Saved',destination)

if __name__=='__main__':main()
