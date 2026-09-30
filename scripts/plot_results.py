#!/usr/bin/env python3
"""Plot the scenario CSVs written by `attitude_sim` into docs/*.png.

    ./build/attitude_sim tumble --out data/tumble_rk4.csv
    ./build/attitude_sim tumble --integrator euler --out data/tumble_euler.csv
    ./build/attitude_sim slew && ./build/attitude_sim detumble && ./build/attitude_sim gravity
    python scripts/plot_results.py
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
DATA, DOCS = ROOT / "data", ROOT / "docs"
DOCS.mkdir(exist_ok=True)
plt.rcParams.update({"figure.dpi": 130, "axes.grid": True, "grid.alpha": 0.3, "axes.spines.top": False,
                     "axes.spines.right": False, "font.size": 10})


def load(name):
    return np.genfromtxt(DATA / f"{name}.csv", delimiter=",", names=True)


def wnorm(d):
    return np.sqrt(d["wx"] ** 2 + d["wy"] ** 2 + d["wz"] ** 2)


def tumble():
    rk4, eul = load("tumble_rk4"), load("tumble_euler")
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
    for k, lab in (("wx", r"$\omega_x$"), ("wy", r"$\omega_y$ (intermediate axis)"), ("wz", r"$\omega_z$")):
        ax[0].plot(rk4["t"], rk4[k], label=lab, lw=1.2)
    ax[0].set(xlabel="time [s]", ylabel="angular velocity [rad/s]", title="Torque-free spin about the intermediate axis")
    ax[0].legend(loc="lower left", fontsize=8)
    for d, lab in ((eul, "explicit Euler"), (rk4, "RK4")):
        drift = np.maximum(np.abs(d["energy"] - d["energy"][0]) / d["energy"][0], 1e-17)
        ax[1].semilogy(d["t"], drift, label=lab, lw=1.2)
    ax[1].set(xlabel="time [s]", ylabel="relative kinetic-energy drift", title="Energy conservation, dt = 0.01 s")
    ax[1].legend()
    fig.tight_layout()
    fig.savefig(DOCS / "tumble.png")


def slew():
    d = load("slew")
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
    ax[0].plot(d["t"], d["pointing_error_deg"], lw=1.4)
    ax[0].set(xlabel="time [s]", ylabel="pointing error [deg]", title="60° slew: pointing error")
    for k in ("tau_x", "tau_y", "tau_z"):
        ax[1].plot(d["t"], d[k] * 1e3, label=k.replace("tau_", r"$\tau_") + "$", lw=1.2)
    ax[1].axhline(2, color="k", ls="--", lw=0.8)
    ax[1].axhline(-2, color="k", ls="--", lw=0.8, label="torque limit")
    ax[1].set(xlabel="time [s]", ylabel="torque [mN·m]", title="Commanded torque (saturated at ±2 mN·m)", xlim=(0, 120))
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(DOCS / "slew.png")


def detumble():
    d = load("detumble")
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
    ax[0].semilogy(d["t"], np.maximum(np.degrees(wnorm(d)), 1e-8), lw=1.4)
    ax[0].set(xlabel="time [s]", ylabel=r"$|\omega|$ [deg/s]", title="Detumble: angular rate", ylim=(1e-6, 10), xlim=(0, 120))
    ax[1].semilogy(d["t"], np.maximum(d["pointing_error_deg"], 1e-8), lw=1.4, color="C1")
    ax[1].set(xlabel="time [s]", ylabel="pointing error [deg]", title="Detumble: attitude error", ylim=(1e-6, 100), xlim=(0, 120))
    fig.tight_layout()
    fig.savefig(DOCS / "detumble.png")


def gravity():
    d = load("gravity")
    fig, ax = plt.subplots(figsize=(6.5, 3.8))
    for k in ("tau_x", "tau_y", "tau_z"):
        ax.plot(d["t"] / 60.0, d[k] * 1e7, label=k.replace("tau_", r"$\tau_") + "$", lw=1.2)
    ax.set(xlabel="time [min]", ylabel=r"torque [$10^{-7}$ N·m]", title="Gravity-gradient torque, 525 km circular orbit")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(DOCS / "gravity.png")


if __name__ == "__main__":
    tumble(), slew(), detumble(), gravity()
    print("wrote", ", ".join(sorted(p.name for p in DOCS.glob("*.png"))))
