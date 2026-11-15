"""
Report figures for DIT7 Assignment 1.

Fig A  fmu_verification.png   FMU vs analytical solution (position + belt speed,
                              stacked panels sharing the time axis - not a dual axis)
Fig B  fmu_convergence.png    detection error vs solver step size (log-log)
Fig C  cell_cycles.png        three consecutive cycles from the offline
                              co-simulation: stage timeline + product position
"""
import numpy as np
from scipy.optimize import brentq
from fmpy import simulate_fmu, extract, read_model_description
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import cell_sim as cs

# --- palette (validated categorical slots 1-3, light surface) ---
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#b8b7b0"
RED = "#e34948"
FMU = "ConveyorTransport.fmu"
v_max, tau, x_sensor = 0.20, 0.30, 0.50

plt.rcParams.update({
    "font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK2,
    "xtick.color": INK2, "ytick.color": INK2, "axes.titlecolor": INK,
    "axes.spines.top": False, "axes.spines.right": False,
    "grid.color": MUTED, "grid.alpha": 0.35, "grid.linewidth": 0.6,
})


def x_analytic(t):
    return v_max * (t - tau * (1 - np.exp(-t / tau)))


# =========================================================================
# Fig A - verification
# =========================================================================
stop, h = 6.0, 0.01
sig = np.array([(0.0, True, False, False), (stop, True, False, False)],
               dtype=[('time', 'f8'), ('BeltRun', 'b'), ('Picked', 'b'), ('LoadNew', 'b')])
res = simulate_fmu(FMU, stop_time=stop, step_size=h, output_interval=h,
                   start_values={'v_max': v_max, 'tau': tau, 'x_sensor': x_sensor},
                   input=sig, output=['BeltSpeed', 'ProductPos', 'ProductAtPick'])
t, pos, spd = res['time'], res['ProductPos'], res['BeltSpeed']
atpick = res['ProductAtPick'].astype(bool)
t_fmu = t[np.argmax(atpick)]
t_anal = brentq(lambda tt: x_analytic(tt) - x_sensor, 0.01, 100.0)

fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.2, 5.4), sharex=True,
                             gridspec_kw={"height_ratios": [2, 1], "hspace": 0.12})

a1.plot(t, pos, color=BLUE, lw=2, label="Product position x(t) — FMU", zorder=3)
a1.plot(t, x_analytic(t), color=INK, lw=1.2, ls=(0, (2, 2)),
        label="Analytical x(t) = v·[t − τ(1 − e^(−t/τ))]", zorder=4)
a1.axhline(x_sensor, color=MUTED, lw=1)
a1.annotate(f"light barrier  x = {x_sensor} m", (0.12, x_sensor), textcoords="offset points",
            xytext=(0, 5), color=INK2, fontsize=8)
a1.axvline(t_anal, color=RED, lw=1, ls=(0, (3, 3)))
a1.annotate(f"arrival  t = {t_anal:.3f} s\nFMU  t = {t_fmu:.3f} s  (−1 step)",
            (t_anal, 0.04), textcoords="offset points", xytext=(8, 0),
            color=RED, fontsize=8, va="bottom")
a1.set_ylabel("position [m]")
a1.set_ylim(0, pos.max() * 1.05)
a1.grid(axis="y")
a1.legend(loc="upper left", frameon=False, fontsize=8)
a1.set_title("ConveyorTransport FMU — verification against the analytical solution", pad=10)

a2.plot(t, spd, color=AQUA, lw=2, zorder=3)
a2.axhline(v_max, color=MUTED, lw=1)
a2.annotate(f"v_max = {v_max} m/s", (0.12, v_max), textcoords="offset points",
            xytext=(0, -12), color=INK2, fontsize=8)
a2.set_ylabel("belt speed [m/s]")
a2.set_xlabel("time [s]")
a2.set_xlim(0, stop)
a2.set_ylim(0, v_max * 1.25)
a2.grid(axis="y")
fig.savefig("fmu_verification.png", dpi=160, bbox_inches="tight",
            facecolor="white")
plt.close(fig)

# =========================================================================
# Fig B - convergence
# =========================================================================
hs = [0.04, 0.02, 0.01, 0.005, 0.0025, 0.001, 0.0005]
errs = []
for hh in hs:
    s = np.array([(0.0, True, False, False), (stop, True, False, False)],
                 dtype=[('time', 'f8'), ('BeltRun', 'b'), ('Picked', 'b'), ('LoadNew', 'b')])
    r = simulate_fmu(FMU, stop_time=stop, step_size=hh, output_interval=hh,
                     start_values={'v_max': v_max, 'tau': tau, 'x_sensor': x_sensor},
                     input=s, output=['ProductAtPick'])
    ap = r['ProductAtPick'].astype(bool)
    errs.append(abs(r['time'][np.argmax(ap)] - t_anal) * 1000)

fig, ax = plt.subplots(figsize=(5.2, 4))
ms = [x * 1000 for x in hs]
ax.loglog(ms, ms, color=MUTED, lw=1.2, ls=(0, (3, 3)), label="slope 1 (error = h)")
ax.loglog(ms, errs, "o-", color=BLUE, lw=2, ms=6, label="detection error", zorder=3)
for xx, yy in zip(ms, errs):
    if xx in (40, 0.5):
        ax.annotate(f"{yy:.2f} ms", (xx, yy), textcoords="offset points",
                    xytext=(6, -10), fontsize=8, color=INK2)
ax.set_xlabel("solver / communication step size h [ms]")
ax.set_ylabel("arrival-time error [ms]")
ax.set_title("Step-size convergence of the detection event", pad=10)
ax.grid(which="both")
ax.legend(frameon=False, fontsize=8, loc="upper left")
fig.savefig("fmu_convergence.png", dpi=160, bbox_inches="tight", facecolor="white")
plt.close(fig)

# =========================================================================
# Fig C - three consecutive cycles from the offline co-simulation
# =========================================================================
unz = extract(cs.FMU_PATH)
md = read_model_description(cs.FMU_PATH)
plant = cs.Plant(unz, md)
cell = cs.CellController(2, cycles=3)

T, STATE, POS, CYC = [], [], [], []
tt = 0.0
while tt < 25 and not cell.Sts_Done:
    cell.scan((tt < 0.05), False, False, True, True,
              plant.AtPick, plant.HasPart, plant.Act, tt)
    plant.step(cell.Cmd_A, cell.Cmd_Grip, cell.Cmd_Band,
               cell.Cmd_LoadNew, cell.Cmd_Picked, tt, cs.DT)
    T.append(tt); STATE.append(cell.State)
    POS.append(plant.ProductPos); CYC.append(cell.Sts_CycleCount)
    tt += cs.DT
plant.fmu.terminate(); plant.fmu.freeInstance()
T = np.array(T); STATE = np.array(STATE); POS = np.array(POS)

STAGES = {10: ("GIVE", BLUE), 20: ("PROCESS", ORANGE), 30: ("PUT AWAY", AQUA)}
spans = []
cur, t0 = STATE[0], T[0]
for i in range(1, len(T)):
    if STATE[i] != cur:
        spans.append((cur, t0, T[i] - t0))
        cur, t0 = STATE[i], T[i]
spans.append((cur, t0, T[-1] - t0))

fig, (b1, b2) = plt.subplots(2, 1, figsize=(7.6, 4.8), sharex=True,
                             gridspec_kw={"height_ratios": [1, 1.6], "hspace": 0.30})

for st, x0, w in spans:
    if st in STAGES:
        name, col = STAGES[st]
        b1.broken_barh([(x0 + 0.02, w - 0.04)], (0.25, 0.5),
                       facecolors=col, edgecolor="white", linewidth=1)
        if w > 1.2:
            b1.text(x0 + w / 2, 0.5, name, ha="center", va="center",
                    color="white", fontsize=7.5, fontweight="bold")
cyc_ends = [T[i] for i in range(1, len(T)) if CYC[i] != CYC[i - 1]]
for i, ce in enumerate(cyc_ends, start=1):
    b1.axvline(ce, color=INK2, lw=0.8, ls=(0, (2, 2)))
    b1.text(ce, 0.12, f"cycle {i}", ha="right", va="top", fontsize=7.5, color=INK2)
b1.set_ylim(0, 1)
b1.set_yticks([])
b1.spines["left"].set_visible(False)
b1.set_title("Three consecutive cycles — offline co-simulation of the PLC logic "
             "against ConveyorTransport.fmu", pad=30)
b1.legend(handles=[Patch(facecolor=c, label=n) for n, c in STAGES.values()],
          loc="lower left", bbox_to_anchor=(0, 1.02), ncol=3,
          frameon=False, fontsize=8)

b2.plot(T, POS, color=BLUE, lw=2, zorder=3)
b2.axhline(x_sensor, color=MUTED, lw=1)
b2.annotate("light barrier  x = 0.5 m", (T[-1] * 0.62, x_sensor),
            textcoords="offset points", xytext=(0, -6), color=INK2, fontsize=8,
            va="top", ha="left")
b2.set_ylabel("product position [m]")
b2.set_xlabel("time [s]")
b2.set_xlim(0, T[-1])
b2.set_ylim(0, max(0.62, POS.max() * 1.1))
b2.grid(axis="y")
fig.savefig("cell_cycles.png", dpi=160, bbox_inches="tight", facecolor="white")
plt.close(fig)

print("cycle end times [s]:", [round(x, 2) for x in cyc_ends])
print("Saved fmu_verification.png, fmu_convergence.png, cell_cycles.png")
