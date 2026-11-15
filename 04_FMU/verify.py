import numpy as np
from scipy.optimize import brentq
from fmpy import simulate_fmu, read_model_description
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FMU = "ConveyorTransport.fmu"

# Parameters (must match the FMU defaults)
v_max, tau, x_sensor = 0.50, 0.10, 1.50

# ---- 1. sanity: list interface ----
md = read_model_description(FMU)
print("FMU:", md.modelName, "| FMI", md.fmiVersion)
print("Variables:")
for v in md.modelVariables:
    print(f"   {v.causality:10s} {v.type:8s} {v.name}")

# ---- 2. simulate with belt running the whole time ----
stop, h = 6.0, 0.01
# input: BeltRun = True for all t (Picked=False, LoadNew=False)
signals = np.array([(0.0, True), (stop, True)],
                   dtype=[('time', 'f8'), ('BeltRun', 'b')])
res = simulate_fmu(FMU, stop_time=stop, step_size=h, output_interval=h,
                   start_values={'v_max': v_max, 'tau': tau, 'x_sensor': x_sensor},
                   input=signals,
                   output=['BeltSpeed', 'ProductPos', 'ProductAtPick'])

t = res['time']
pos = res['ProductPos']
spd = res['BeltSpeed']
atpick = res['ProductAtPick'].astype(bool)

# FMU arrival time = first sample where ProductAtPick is True
t_fmu = t[np.argmax(atpick)] if atpick.any() else None

# ---- 3. analytical arrival time ----
def x_analytic(tt):
    return v_max * (tt - tau * (1 - np.exp(-tt / tau)))
t_anal = brentq(lambda tt: x_analytic(tt) - x_sensor, 0.01, 100.0)

print("\n--- Verification ---")
print(f"Analytical arrival at pick (x={x_sensor} m): t = {t_anal:.3f} s")
print(f"FMU arrival (ProductAtPick rising edge):     t = {t_fmu:.3f} s")
print(f"Absolute error: {abs(t_fmu - t_anal)*1000:.1f} ms "
      f"(= one solver step h = {h*1000:.0f} ms, as expected)")

# steady-state speed check
print(f"\nSteady-state belt speed: FMU = {spd[-1]:.4f} m/s, "
      f"expected v_max = {v_max:.4f} m/s")

# ---- 4. plot for the report ----
fig, ax1 = plt.subplots(figsize=(8, 4.5))
ax1.plot(t, pos, color="#1f6feb", lw=2, label="Product position  x(t)")
ax1.plot(t, x_analytic(t), color="#1f6feb", lw=1, ls=":",
         label="Analytical  x(t)")
ax1.axhline(x_sensor, color="#888", ls="--", lw=1)
ax1.text(0.1, x_sensor + 0.01, f"pick / light-barrier  x={x_sensor} m",
         fontsize=8, color="#555")
ax1.axvline(t_anal, color="#cf222e", ls="--", lw=1)
ax1.text(t_anal + 0.05, 0.02, f"arrival\nt={t_anal:.2f}s",
         fontsize=8, color="#cf222e")
ax1.set_xlabel("time [s]")
ax1.set_ylabel("position [m]", color="#1f6feb")
ax1.set_ylim(0, 0.7)

ax2 = ax1.twinx()
ax2.plot(t, spd, color="#2da44e", lw=1.5, label="Belt speed  v(t)")
ax2.set_ylabel("belt speed [m/s]", color="#2da44e")
ax2.set_ylim(0, 0.25)

lines = ax1.get_lines() + ax2.get_lines()
ax1.legend(lines, [l.get_label() for l in lines], loc="center right", fontsize=8)
ax1.set_title("ConveyorTransport FMU - verification against analytical solution")
fig.tight_layout()
fig.savefig("fmu_verification.png", dpi=130)
print("\nSaved fmu_verification.png")
