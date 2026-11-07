"""Networked navigation: navigator + local planner on a SERVER, the robot behind a Wi-Fi link.

Time base: the robot loop runs at 20 Hz (substeps of 50 ms), the server at 10 Hz (every
second substep), like run_episode's control_dt = 0.1. All delays are multiples of 50 ms.

Link model (both directions share one channel state):
  * constant one-way delay `delay` (uplink and downlink each; RTT = 2 * delay);
  * i.i.d. packet loss `loss`, independent per packet and direction;
  * Gilbert-Elliott bursts: the channel is Good/Bad, in Bad every packet is lost.
    Mean Bad length `ge_len` s, fraction of time in Bad `ge_frac`;
  * optional deterministic outage of `ev_len` s that starts when the first box drops
    (or at `ev_open_t` s in a scenario without boxes): Wi-Fi dropout at the worst moment.
No retransmissions (UDP-like); a lost scan is lost for the server.

Messages
  uplink, 10 Hz:   stamp, pose, velocity, path ack; the lidar scan (5 Hz) piggybacks
  downlink, after every server tick that got fresh state: state stamp + command(s)

Robot strategies
  S0  hold the last received command forever
  S1  hold the last command up to 0.5 s after the last packet, then stop (Nav2 cmd_vel timeout)
  S2  trajectory buffer: the server sends the whole optimal control sequence (MPPI U) stamped
      with the time of the state it was planned from; the robot plays it at the right index
      and stops when it runs out (DWA/RPP: the command repeated over the planner horizon)
  S3  S2 + onboard safety: the next 0.5 s of the buffered motion is rolled out on the robot's
      own fresh lidar costmap; stop if it would collide
  S4  S3 + onboard fallback planner: no packet for > 0.3 s -> a cheap planner on the robot
      (DWA, or MPPI with few samples) follows the last received global path on the robot's
      costmap; control returns to the server as soon as a packet arrives
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field

import mujoco
import numpy as np

from mpp.geometry import rollout_unicycle
from drive import DriveModel, LaggedDrive, ModelMPPI, model_rollout
from mpp.mapping.costmap import Costmap
from mpp.planning.local_planners import make_local_planner
from mpp.robot.model import BURGER
from mpp.runner import EpisodeConfig, make_episode

SUB = 0.05  # robot substep, s
HDR = 42  # bytes per packet: IP+UDP (28) + seq/stamp/type (14)
STATE_B = 20  # pose (3 x f32) + velocity (2 x f32)
CMD_B = 8  # one (v, w) pair in f32
EPS = 1e-6


@dataclass
class LinkConfig:
    delay: float = 0.0  # one-way, s
    loss: float = 0.0  # i.i.d. loss probability per packet
    ge_len: float = 0.0  # mean outage length, s (0 = no bursts)
    ge_frac: float = 0.0  # fraction of time in outage
    ev_len: float = 0.0  # deterministic outage length starting at the first box drop
    ev_open_t: float = 8.0  # ... or at this time if the scenario has no boxes

    def label(self) -> str:
        s = f"d={int(round(self.delay * 1000))}ms"
        if self.loss:
            s += f" p={int(round(self.loss * 100))}%"
        if self.ge_len:
            s += f" GE{self.ge_len:g}s/{int(round(self.ge_frac * 100))}%"
        if self.ev_len:
            s += f" drop@box{self.ev_len:g}s"
        return s


@dataclass
class NetConfig:
    strategy: str = "S0"
    link: LinkConfig = field(default_factory=LinkConfig)
    hold_timeout: float = 0.5  # S1
    safety_horizon: float = 0.5  # S3/S4
    safety_margin: float = 0.01  # S3/S4: clearance below radius + margin is a collision
    fallback_after: float = 0.3  # S4
    fallback_planner: str = "dwa"  # S4: "dwa" or "mppi" (with fallback_samples)
    fallback_samples: int = 16
    const_horizon: float | None = None  # S2 for DWA/RPP: length of the repeated command
    drain: float = 4.0  # s after the server is done, wait for the robot to stop
    server_every: int = 1  # the server plans on every n-th 10 Hz tick (10 / n Hz)
    shift_aligned: bool = False  # warm start shifted by the real time between plans
    # Adaptive server rate (overrides server_every). On a tick with fresh state the server
    # replans only if one of the cheap checks fires, otherwise the robot keeps its buffer:
    #   * `adapt_max_every` ticks passed since the last plan;
    #   * the reported pose is more than `adapt_dev` m away from where the last plan put it;
    #   * the rest of the buffer, rolled out from the reported pose for `adapt_look` s on the
    #     server costmap, comes closer than `adapt_near` m (beyond the radius) to an obstacle;
    #   * the buffer is about to run out or is a stop/backup command.
    adaptive: bool = False
    adapt_max_every: int = 4
    adapt_near: float = 0.25
    adapt_look: float = 1.0
    adapt_dev: float = 0.05
    # Model mismatch (drive.py): the true drive of the robot, and the model the planner and
    # the robot/server checks use (None = the plain unicycle of mpp). Tuples (tau_v, tau_w, k_v, k_w).
    plant: tuple | None = None
    planner_model: tuple | None = None
    log_drive: bool = False  # return (t, v_cmd, w_cmd, v_meas, w_meas) per substep


class Channel:
    """Shared Gilbert-Elliott state + i.i.d. loss, advanced once per substep."""

    def __init__(self, cfg: LinkConfig, rng: np.random.Generator) -> None:
        self.cfg, self.rng = cfg, rng
        self.bad = False
        self.ev_start: float | None = None
        if cfg.ge_len > 0 and cfg.ge_frac > 0:
            good_len = cfg.ge_len * (1 - cfg.ge_frac) / cfg.ge_frac
            self.p_gb = SUB / good_len
            self.p_bg = SUB / cfg.ge_len
        else:
            self.p_gb = self.p_bg = 0.0
        self.bad_time = 0.0

    def advance(self, t: float) -> None:
        if self.bad:
            self.bad = self.rng.random() >= self.p_bg
        else:
            self.bad = self.rng.random() < self.p_gb

    def down(self, t: float) -> bool:
        if self.bad:
            return True
        c = self.cfg
        return bool(c.ev_len and self.ev_start is not None and self.ev_start <= t < self.ev_start + c.ev_len)

    def lost(self, t: float) -> bool:
        if self.down(t):
            return True
        return self.cfg.loss > 0 and self.rng.random() < self.cfg.loss


def _substep(sim, update_boxes: bool) -> None:
    """sim.step(SUB), but boxes are only checked at the 10 Hz server ticks like run_episode."""
    n = int(round(SUB / sim.model.opt.timestep))
    for _ in range(n):
        mujoco.mj_step(sim.model, sim.data)
        sim._track_contacts()
    if update_boxes:
        sim._update_boxes()


def run_net_episode(cfg: EpisodeConfig, net: NetConfig) -> dict:
    wall0 = time.perf_counter()
    layout, scenario, start, goal, sim, cm_srv, nav = make_episode(cfg)
    pm = DriveModel(*net.planner_model) if net.planner_model is not None else None
    if pm is not None and type(nav.lp).__name__ == "MPPI":
        nav.lp.__class__ = ModelMPPI
        nav.lp.model = pm
    drive = LaggedDrive(DriveModel(*net.plant), SUB) if net.plant is not None else None
    drive_log = []
    robot = BURGER
    strat = net.strategy
    link = net.link
    rng = np.random.default_rng(10_000 + cfg.seed)
    chan = Channel(link, rng)
    dt = cfg.control_dt
    ticks_per_srv = int(round(dt / SUB))
    d_steps = int(round(link.delay / SUB))
    assert abs(d_steps * SUB - link.delay) < 1e-9, "delay must be a multiple of 50 ms"

    onboard_map = strat in ("S3", "S4")
    cm_rob = Costmap.from_layout(layout, robot_radius=robot.radius) if onboard_map else None

    # bootstrap: mission + initial state + first scan delivered reliably at t = 0
    s0 = sim.scan()
    cm_srv.integrate_scan(s0.origin, s0.angles, s0.ranges, s0.max_range)
    if cm_rob is not None:
        cm_rob.integrate_scan(s0.origin, s0.angles, s0.ranges, s0.max_range)
    nav.start(sim.pose(), sim.time)

    # onboard fallback planner (S4)
    fb = None
    if strat == "S4":
        kw = {}
        if net.fallback_planner == "mppi":
            kw = dict(samples=net.fallback_samples, seed=cfg.seed + 777)
        fb = make_local_planner(net.fallback_planner, robot, dt=dt, **kw)
    rob_path_ver = -1  # robot's copy of the global path (S4)
    srv_path_ver = 0
    srv_paths_seen = len(nav.stats.plans)
    acked_ver = -1  # what the server knows the robot has

    up_q: list = []  # (arrival_step, seq, msg)
    down_q: list = []
    seq_no = 0
    bytes_up = bytes_down = 0
    pk_up = pk_down = lost_up = lost_down = 0

    # robot-side state
    buf = None  # (stamp, seq array (n,2), step dt)
    last_cmd = (0.0, 0.0)
    last_rx: float | None = None
    have_cmd = False
    latest_srv_stamp = -1.0
    fb_active = False
    fb_cmd = (0.0, 0.0)
    fb_time = 0.0
    fb_entries = 0
    fb_ms = []
    safety_stops = 0
    safety_time = 0.0
    safety_ms = []
    hold_time = 0.0  # robot outputs zero because it has no valid command
    stopped_time = 0.0
    outage_time = 0.0

    srv_status = None  # goal | failed
    srv_end_t = None
    srv_latest_state = None  # (stamp, pose, vel)
    srv_new_state = False
    next_scan = cfg.scan_period
    traj = [(*sim.pose(), 0.0, 0.0, sim.time)]
    known_drops: set[str] = set()
    step_i = 0
    max_steps = int(round(cfg.max_time / SUB))
    T_mppi = getattr(nav.lp, "T", None)
    const_h = net.const_horizon or getattr(nav.lp, "steps", None) and nav.lp.steps * dt or 1.0

    def send(q, arrival_step, msg, nbytes, up: bool):
        nonlocal seq_no, bytes_up, bytes_down, pk_up, pk_down, lost_up, lost_down
        tnow = step_i * SUB
        if up:
            bytes_up += nbytes
            pk_up += 1
        else:
            bytes_down += nbytes
            pk_down += 1
        if chan.lost(tnow):
            if up:
                lost_up += 1
            else:
                lost_down += 1
            return
        seq_no += 1
        heapq.heappush(q, (arrival_step, seq_no, msg))

    srv_tick_i = 0
    srv_plan = None  # (stamp, pose, seq, regular MPPI plan?, server tick) of the last plan sent
    why = dict(first=0, period=0, dev=0, near=0, short=0, other=0, skip=0)
    check_ms = []
    plan_err = []  # reported pose vs where the last plan put it, m (every fresh state)
    n_look = max(int(round(net.adapt_look / dt)), 1)

    def need_plan() -> bool:
        """Cheap server-side check: does this robot need a new MPPI plan on this tick?"""
        t0 = time.perf_counter()
        r = _need_plan()
        check_ms.append(1000 * (time.perf_counter() - t0))
        why[r] += 1
        return r != "skip"

    def _need_plan() -> str:
        if srv_plan is None:
            return "first"
        p_stamp, p_pose, p_vel, p_seq, regular, p_tick = srv_plan
        if srv_status is not None or not regular:
            return "other"  # done, or the last command was a stop / backup / constant one
        if srv_tick_i - p_tick >= net.adapt_max_every:
            return "period"
        stamp, pose, vel_s = srv_latest_state
        k = int(math.floor((stamp - p_stamp) / dt + EPS))
        if k + n_look > len(p_seq):
            return "short"
        if k > 0:
            pred = model_rollout(p_pose, p_seq[None, :k, 0], p_seq[None, :k, 1], dt, p_vel, pm)[0, -1]
            if math.hypot(pred[0] - pose[0], pred[1] - pose[1]) > net.adapt_dev:
                return "dev"
        u = np.repeat(p_seq[k:k + n_look], 2, axis=0)  # 50 ms steps
        tr = model_rollout(pose, u[None, :, 0], u[None, :, 1], dt / 2, vel_s, pm)[0]
        if cm_srv.clearance_at(tr[:, :2]).min() - robot.radius < net.adapt_near:
            return "near"
        return "skip"

    while step_i < max_steps:
        t = step_i * SUB
        tick = step_i % ticks_per_srv == 0
        chan.advance(t)
        if chan.down(t):
            outage_time += SUB

        # ---------------- robot: sense and send state (10 Hz, scan 5 Hz)
        if tick:
            scan = None
            if t + 1e-9 >= next_scan:
                scan = sim.scan()
                next_scan += cfg.scan_period
                if cm_rob is not None:
                    cm_rob.integrate_scan(scan.origin, scan.angles, scan.ranges, scan.max_range)
            msg = dict(stamp=t, pose=sim.pose(), vel=sim.velocity(), scan=scan, ack=rob_path_ver)
            nb = HDR + STATE_B + (4 * len(scan.ranges) if scan is not None else 0)
            send(up_q, step_i + d_steps, msg, nb, up=True)

        # ---------------- server (10 Hz)
        if tick:
            while up_q and up_q[0][0] <= step_i:
                _, _, m = heapq.heappop(up_q)
                if m["scan"] is not None:
                    s = m["scan"]
                    cm_srv.integrate_scan(s.origin, s.angles, s.ranges, s.max_range)
                if srv_latest_state is None or m["stamp"] > srv_latest_state[0]:
                    srv_latest_state = (m["stamp"], m["pose"], m["vel"])
                    srv_new_state = True
                acked_ver = max(acked_ver, m["ack"])
            if srv_new_state and srv_plan is not None and srv_plan[4]:
                p_stamp, p_pose, p_vel, p_seq = srv_plan[:4]
                st_, ps_, _ = srv_latest_state
                kk = int(math.floor((st_ - p_stamp) / dt + EPS))
                if 0 < kk <= len(p_seq):
                    pr = model_rollout(p_pose, p_seq[None, :kk, 0], p_seq[None, :kk, 1], dt, p_vel, pm)[0, -1]
                    plan_err.append(math.hypot(pr[0] - ps_[0], pr[1] - ps_[1]))
            if net.adaptive:
                srv_turn = srv_new_state and need_plan()
            else:
                srv_turn = (step_i // ticks_per_srv) % net.server_every == 0
            srv_tick_i += 1
            if srv_new_state and srv_turn:
                srv_new_state = False
                if net.shift_aligned and net.server_every > 1 and T_mppi is not None:
                    # compute() shifts by one step; shift by the rest of the elapsed time
                    k = net.server_every - 1
                    nav.lp.U = np.vstack([nav.lp.U[k:], np.repeat(nav.lp.U[-1:], k, axis=0)])
                stamp, pose, vel = srv_latest_state
                if srv_status is None:
                    calls = nav.stats.local_calls
                    v, w = nav.tick(pose, vel, t)
                    if nav.state in ("done", "failed"):
                        srv_status = "goal" if nav.state == "done" else "failed"
                        srv_end_t = t
                        v, w = 0.0, 0.0
                else:
                    calls = nav.stats.local_calls
                    v, w = 0.0, 0.0
                if len(nav.stats.plans) != srv_paths_seen:
                    srv_paths_seen = len(nav.stats.plans)
                    srv_path_ver += 1
                from_lp = (
                    srv_status is None
                    and nav.stats.local_calls > calls
                    and nav.last_local is not None
                    and nav.last_local.v == v
                    and nav.last_local.w == w
                )
                if strat in ("S2", "S3", "S4"):
                    if from_lp and T_mppi is not None and nav.last_local.status == "ok":
                        seq = nav.lp.U.copy()
                        seq[0] = (v, w)
                    elif from_lp and (v != 0.0 or w != 0.0):
                        seq = np.tile([v, w], (max(int(round(const_h / dt)), 1), 1))
                    else:
                        # backup / stop: 0.5 s after the expected arrival; the server knows
                        # the state's age, assumes a symmetric link
                        age = t - stamp
                        seq = np.tile([v, w], (max(int(round((2 * age + 0.5) / dt)), 5), 1))
                else:
                    seq = np.array([[v, w]])
                msg = dict(stamp=stamp, seq=seq, path=None, ver=srv_path_ver)
                nb = HDR + CMD_B * len(seq)
                if strat == "S4" and acked_ver < srv_path_ver and nav.path is not None:
                    msg["path"] = nav.lp.path.copy()
                    nb += 8 * len(nav.lp.path) // 4  # path decimated to 20 cm when sent
                send(down_q, step_i + d_steps, msg, nb, up=False)
                regular = from_lp and T_mppi is not None and nav.last_local.status == "ok"
                srv_plan = (stamp, pose, vel, seq, regular, srv_tick_i)

        # ---------------- robot: receive commands
        got = False
        while down_q and down_q[0][0] <= step_i:
            _, _, m = heapq.heappop(down_q)
            got = True
            last_rx = t
            if m["path"] is not None and m["ver"] > rob_path_ver:
                rob_path_ver = m["ver"]
                fb.set_path(m["path"])
            if m["stamp"] >= latest_srv_stamp:
                latest_srv_stamp = m["stamp"]
                buf = (m["stamp"], m["seq"])
                last_cmd = (float(m["seq"][0, 0]), float(m["seq"][0, 1]))
                have_cmd = True

        # ---------------- robot: choose the command
        pose_r, vel_r = sim.pose(), sim.velocity()
        if fb is not None and fb.path is not None and tick:
            fb.update_progress(pose_r)
        since = math.inf if last_rx is None else t - last_rx
        holding = False
        if strat == "S0":
            cmd = last_cmd
            holding = not have_cmd
        elif strat == "S1":
            if have_cmd and since <= net.hold_timeout + EPS:
                cmd = last_cmd
            else:
                cmd, holding = (0.0, 0.0), True
        else:
            cmd, holding = (0.0, 0.0), True
            seq_rest = None
            if buf is not None:
                k = int(math.floor((t - buf[0]) / dt + EPS))
                if 0 <= k < len(buf[1]):
                    cmd, holding = (float(buf[1][k, 0]), float(buf[1][k, 1])), False
                    seq_rest = buf[1][k:]
            use_fb = (
                strat == "S4"
                and fb.path is not None
                and srv_status is None
                and last_rx is not None
                and since > net.fallback_after + EPS
            )
            if use_fb:
                if not fb_active:
                    fb_active, fb_entries = True, fb_entries + 1
                if tick:
                    if np.linalg.norm(np.array(pose_r[:2]) - fb.path[-1]) <= nav.cfg.goal_tolerance:
                        fb_cmd = (0.0, 0.0)
                    else:
                        t0 = time.perf_counter()
                        lp = fb.compute(pose_r, vel_r, cm_rob)
                        fb_ms.append(1000 * (time.perf_counter() - t0))
                        fb_cmd = (lp.v, lp.w)
                cmd, holding = fb_cmd, False
                fb_time += SUB
                seq_rest = None  # the fallback planner checks its own rollouts
            else:
                fb_active = False
            if strat in ("S3", "S4") and seq_rest is not None and (cmd[0] != 0.0):
                t0 = time.perf_counter()
                n = int(round(net.safety_horizon / dt))
                u = seq_rest[:n]
                if len(u) < n:
                    u = np.vstack([u, np.zeros((n - len(u), 2))])
                # roll out at 25 ms resolution so a 10 cm cell is not skipped at 0.6 m/s
                u = np.repeat(u, 4, axis=0)
                tr = model_rollout(pose_r, u[None, :, 0], u[None, :, 1], dt / 4, vel_r, pm)[0]
                now_c = float(cm_rob.clearance_at(np.array(pose_r[:2])))
                thr = min(robot.radius + net.safety_margin, now_c - 0.005)
                if (cm_rob.clearance_at(tr[:, :2]) < thr).any():
                    cmd = (0.0, 0.0)
                    safety_time += SUB
                    if tick:
                        safety_stops += 1
                safety_ms.append(1000 * (time.perf_counter() - t0))
        if holding:
            hold_time += SUB
        if srv_status is None and abs(vel_r[0]) < 0.01 and abs(vel_r[1]) < 0.05:
            stopped_time += SUB
        if net.log_drive:
            drive_log.append((t, cmd[0], cmd[1], vel_r[0], vel_r[1]))
        sim.set_cmd(*(drive(*cmd) if drive is not None else cmd))

        # ---------------- physics
        _substep(sim, update_boxes=(step_i + 1) % ticks_per_srv == 0)
        step_i += 1
        for name in sim.dropped.keys() - known_drops:
            known_drops.add(name)
            box = sim.boxes[name]
            nav._event(sim.time, "box", f"упала коробка {name}", (box.x, box.y))
            if chan.ev_start is None:
                chan.ev_start = sim.time
        if not sim.boxes and chan.ev_start is None:
            chan.ev_start = link.ev_open_t
        x, y, yaw = sim.pose()
        traj.append((x, y, yaw, cmd[0], cmd[1], sim.time))

        if srv_status is not None:
            vv = sim.velocity()
            stopped = abs(vv[0]) < 0.01 and abs(vv[1]) < 0.05 and cmd == (0.0, 0.0)
            if (stopped and t - srv_end_t > 2 * link.delay) or t - srv_end_t > net.drain:
                break
    sim.set_cmd(0.0, 0.0)
    traj = np.array(traj)

    truth = Costmap.from_layout(layout, robot_radius=robot.radius)
    boxes = sim.box_positions()
    for name, xy in boxes.items():
        b = sim.boxes[name]
        if name in sim.dropped:
            truth.fill_rect(
                xy[0] - b.size[0] / 2, xy[1] - b.size[1] / 2, xy[0] + b.size[0] / 2, xy[1] + b.size[1] / 2
            )
    truth.update()
    gap = float(truth.clearance_at(traj[:, :2]).min() - robot.radius)
    goal_err = float(np.linalg.norm(traj[-1, :2] - goal.xy))
    status = srv_status or "timeout"
    reached = status == "goal" and goal_err <= 0.3
    dur = max(sim.time, 1e-6)
    st = nav.stats
    return dict(
        scenario=cfg.scenario,
        planner=cfg.local_planner,
        seed=cfg.seed,
        strategy=strat,
        link=link.label(),
        delay=link.delay,
        loss=link.loss,
        ge_len=link.ge_len,
        ge_frac=link.ge_frac,
        ev_len=link.ev_len,
        status=status,
        reached=bool(reached),
        success=bool(reached and sim.contacts.events == 0),
        collisions=int(sim.contacts.events),
        contact_time=round(sim.contacts.time, 3),
        min_gap=round(gap, 3),
        time=round(srv_end_t, 2) if status == "goal" else float("nan"),
        sim_time=round(sim.time, 2),
        goal_err=round(goal_err, 3),
        stopped_time=round(stopped_time, 2),
        hold_time=round(hold_time, 2),
        outage_time=round(outage_time, 2),
        up_Bps=round(bytes_up / dur, 1),
        down_Bps=round(bytes_down / dur, 1),
        lost_up=lost_up,
        lost_down=lost_down,
        pk_up=pk_up,
        pk_down=pk_down,
        replans=st.replans,
        recoveries=st.recoveries,
        safety_stops=safety_stops,
        safety_time=round(safety_time, 2),
        safety_ms=round(float(np.mean(safety_ms)), 3) if safety_ms else 0.0,
        fb_entries=fb_entries,
        fb_time=round(fb_time, 2),
        fb_ms=round(float(np.mean(fb_ms)), 3) if fb_ms else 0.0,
        srv_ms=round(1000 * st.local_time / max(st.local_calls, 1), 3),
        srv_calls=int(st.local_calls),
        server_every=net.server_every,
        shift_aligned=bool(net.shift_aligned),
        adaptive=bool(net.adaptive),
        adapt=dict(max_every=net.adapt_max_every, near=net.adapt_near, look=net.adapt_look,
                   dev=net.adapt_dev) if net.adaptive else None,
        why=why if net.adaptive else None,
        check_ms=round(float(np.mean(check_ms)), 4) if check_ms else 0.0,
        plan_err=round(float(np.mean(plan_err)), 4) if plan_err else None,
        plan_err95=round(float(np.percentile(plan_err, 95)), 4) if plan_err else None,
        plant=list(net.plant) if net.plant is not None else None,
        planner_model=list(net.planner_model) if net.planner_model is not None else None,
        drive_log=drive_log if net.log_drive else None,
        wall=round(time.perf_counter() - wall0, 2),
    )
