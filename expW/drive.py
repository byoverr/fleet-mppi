"""Drive dynamics for the model-mismatch experiment (variant V3 / essay).

The planner in `mpp` assumes that a commanded (v, w) is reached almost at once (unicycle with
acceleration clipping). A heavier robot on a slippery floor does not do that. We model the
real drive as a first-order lag on each channel plus a gain error:

    v_eff' = (k_v * v_cmd - v_eff) / tau_v
    w_eff' = (k_w * w_cmd - w_eff) / tau_w

`LaggedDrive` applies this to the simulated robot (the "true" plant). `model_rollout` uses
the same structure inside MPPI and in the robot/server checks when the planner is given an
(identified) model. `identify` fits (tau, k) per channel by least squares from a log of
commands and measured velocities.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

import mpp.planning.local_planners.mppi as mppi_mod
from mpp.geometry import rollout_unicycle
from mpp.planning.local_planners.mppi import MPPI


@dataclass(frozen=True)
class DriveModel:
    tau_v: float
    tau_w: float
    k_v: float = 1.0
    k_w: float = 1.0

    def as_tuple(self):
        return (self.tau_v, self.tau_w, self.k_v, self.k_w)


class LaggedDrive:
    """The true drive of the simulated robot: applied on every robot substep."""

    def __init__(self, model: DriveModel, sub: float) -> None:
        self.m = model
        self.av = 1.0 - math.exp(-sub / model.tau_v)
        self.aw = 1.0 - math.exp(-sub / model.tau_w)
        self.v = 0.0
        self.w = 0.0

    def __call__(self, v_cmd: float, w_cmd: float) -> tuple[float, float]:
        self.v += self.av * (self.m.k_v * v_cmd - self.v)
        self.w += self.aw * (self.m.k_w * w_cmd - self.w)
        return self.v, self.w


def effective_velocities(v, w, dt, vel0, model: DriveModel):
    """Commands (K, T) -> mean effective velocities over each step (K, T) under the lag."""
    av = 1.0 - math.exp(-dt / model.tau_v)
    aw = 1.0 - math.exp(-dt / model.tau_w)
    K, T = v.shape
    ve = np.empty_like(v)
    we = np.empty_like(w)
    pv = np.full(K, float(vel0[0]))
    pw = np.full(K, float(vel0[1]))
    for t in range(T):
        nv = pv + av * (model.k_v * v[:, t] - pv)
        nw = pw + aw * (model.k_w * w[:, t] - pw)
        ve[:, t] = 0.5 * (pv + nv)
        we[:, t] = 0.5 * (pw + nw)
        pv, pw = nv, nw
    return ve, we


def model_rollout(pose, v, w, dt, vel0, model: DriveModel | None):
    """Like rollout_unicycle, but through the drive model when one is given."""
    if model is None:
        return rollout_unicycle(pose, v, w, dt)
    ve, we = effective_velocities(np.asarray(v, float), np.asarray(w, float), dt, vel0, model)
    return rollout_unicycle(pose, ve, we, dt)


class ModelMPPI(MPPI):
    """mpp's MPPI with its rollouts going through a drive model. The sampling, costs and
    update are unchanged: only the prediction of where a command sequence takes the robot."""

    model: DriveModel | None = None

    def compute(self, pose, vel, costmap):
        if self.model is None:
            return super().compute(pose, vel, costmap)
        orig = mppi_mod.rollout_unicycle
        m = self.model

        def patched(p, v, w, dt):
            return model_rollout(p, v, w, dt, vel, m)

        mppi_mod.rollout_unicycle = patched
        try:
            return super().compute(pose, vel, costmap)
        finally:
            mppi_mod.rollout_unicycle = orig


def _fit_channel(u, y, sub):
    """y[k+1] = a*y[k] + b*u[k]  ->  tau = -sub/ln(a), k = b/(1-a)."""
    A = np.stack([y[:-1], u[:-1]], axis=1)
    (a, b), *_ = np.linalg.lstsq(A, y[1:], rcond=None)
    a = float(np.clip(a, 1e-6, 0.999999))
    tau = -sub / math.log(a)
    k = b / (1.0 - a)
    resid = y[1:] - A @ np.array([a, b])
    return tau, float(k), float(np.sqrt(np.mean(resid**2)))


def _simulate(u, y0, tau, k, sub):
    a = math.exp(-sub / tau)
    y = np.empty_like(u)
    p = y0
    for i in range(len(u)):
        y[i] = p
        p = a * p + (1 - a) * k * u[i]
    return y


def _fit_channel_oe(u, y, sub):
    """Output error: simulate the model from the commands only and fit (tau, k) to the
    measured response. Unlike the one-step regression it is not biased by measurement noise."""
    from scipy.optimize import least_squares

    def res(p):
        return _simulate(u, y[0], p[0], p[1], sub) - y

    tau0, k0, _ = _fit_channel(u, y, sub)
    sol = least_squares(res, x0=[max(tau0, 0.05), k0], bounds=([0.01, 0.1], [5.0, 2.0]))
    return float(sol.x[0]), float(sol.x[1]), float(np.sqrt(np.mean(sol.fun**2)))


def identify(log: np.ndarray, sub: float, method: str = "oe") -> tuple[DriveModel, dict]:
    """log rows: (t, v_cmd, w_cmd, v_meas, w_meas) at the robot substep `sub`.
    method: "oe" (output error, default) or "arx" (one-step least squares)."""
    fit = _fit_channel_oe if method == "oe" else _fit_channel
    tv, kv, rv = fit(log[:, 1], log[:, 3], sub)
    tw, kw, rw = fit(log[:, 2], log[:, 4], sub)
    return DriveModel(tv, tw, kv, kw), dict(rmse_v=rv, rmse_w=rw, n=len(log))
