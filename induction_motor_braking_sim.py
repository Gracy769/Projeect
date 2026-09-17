"""
Phase Induction Motor — Regenerative Braking Simulation (Window 3)
====================================================================
Extends the Eddy-Current Deceleration project with a THIRD braking
mechanism: the locomotive's own 3-phase induction traction motors,
operated in their regenerative-braking (negative-slip) region.

Total deceleration = conventional friction + eddy-current add-on
                      + induction-motor regen contribution

This window shows:
  - Live torque-slip curve (Kloss's equation) with the operating point
    moving through MOTOR -> SYNC -> REGEN as the train slows down
  - A slider for "target slip" (how hard the traction control system
    pushes the motor into regen) -- more negative slip = more braking
    torque (up to breakdown), and shows the trade-off with heat/limits
  - v-t comparison: conventional vs conventional+motor-regen
  - Regenerated energy fed back to the catenary/grid, CO2 saved, and
    homes-powered -- using the SAME accounting style as the eddy window

Run:  python induction_motor_braking_sim.py
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, TextBox
from matplotlib.gridspec import GridSpec

from motor_physics import (
    POLES, SLIP_MAX_TORQUE, WHEEL_RADIUS, GEAR_RATIO, MOTORS_PER_TRAIN,
    synchronous_speed_rpm, wheel_speed_rpm, kloss_torque,
    optimal_inverter_freq_for_braking, motor_braking_force, regen_power_watts,
)

# ---------------------------------------------------------------
# Shared constants (match the eddy-current project's conventions)
# ---------------------------------------------------------------
REGEN_EFFICIENCY = 0.85     # electrical conversion efficiency, motor regen
                             # (higher than eddy's 0.50 -- direct electrical
                             #  path, no mechanical->heat->recapture step)
CO2_PER_KWH = 0.233          # kg CO2 per kWh (India grid avg)
HOME_KWH_PER_DAY = 8.0

PRESETS = {
    # t_max = breakdown torque PER MOTOR (N.m), typical for a traction
    # induction motor of this class (~1500-2200 Nm). Chosen so the
    # motor's regen contribution supplements, rather than dominates,
    # conventional friction braking -- matching how real EMUs behave.
    "INDIAN":  {"speed": 160, "mass": 400, "decel": 0.8, "t_max": 1800, "body_color": "#1f6feb"},
    "BULLET":  {"speed": 200, "mass": 400, "decel": 1.0, "t_max": 2200, "body_color": "#8b1a1a"},
}


# ---------------------------------------------------------------
# Physics: build the full braking run with motor regen included
# ---------------------------------------------------------------
def build_run(v0_kmh, mass_t, conv_decel, t_max, target_slip, n=300):
    """
    Simulate deceleration where, at every instant, the induction motors
    are commanded to hold `target_slip` (the traction controller's regen
    setpoint) until the resulting motor braking force is folded in as an
    ADDITIONAL deceleration term alongside conventional friction.

    Returns a dict of time-series arrays plus summary scalars.
    """
    v0 = v0_kmh / 3.6
    mass_kg = mass_t * 1000.0

    # Baseline (conventional-only) stop time, for choosing dt
    t_stop_conv = v0 / conv_decel
    t_grid = np.linspace(0, t_stop_conv * 1.3, n)  # a bit longer since motor speeds up stop

    v = np.zeros_like(t_grid)
    F_motor = np.zeros_like(t_grid)
    slip_arr = np.zeros_like(t_grid)
    regen_power = np.zeros_like(t_grid)
    v[0] = v0

    dt = t_grid[1] - t_grid[0]
    for i in range(1, len(t_grid)):
        v_prev = v[i - 1]
        if v_prev <= 0.05:
            v[i:] = 0.0
            break

        f_req = optimal_inverter_freq_for_braking(v_prev, target_slip, POLES, WHEEL_RADIUS, GEAR_RATIO)
        s, T, F, regen = motor_braking_force(v_prev, f_req, t_max, MOTORS_PER_TRAIN,
                                              WHEEL_RADIUS, GEAR_RATIO, POLES, SLIP_MAX_TORQUE)
        slip_arr[i - 1] = s
        F_motor[i - 1] = F

        nr_rpm = wheel_speed_rpm(v_prev, WHEEL_RADIUS, GEAR_RATIO)
        regen_power[i - 1] = regen_power_watts(T, nr_rpm, MOTORS_PER_TRAIN)

        total_decel = conv_decel + F / mass_kg
        v[i] = max(0.0, v_prev - total_decel * dt)

    # Trim trailing zeros after stop for cleaner plotting/metrics
    stop_idx = np.argmax(v <= 0.0) if np.any(v <= 0.0) else len(v) - 1
    stop_idx = max(stop_idx, 1)

    t = t_grid[:stop_idx + 1]
    v = v[:stop_idx + 1]
    d = np.concatenate(([0], np.cumsum((v[:-1] + v[1:]) / 2 * dt)))
    F_motor = F_motor[:stop_idx + 1]
    slip_arr = slip_arr[:stop_idx + 1]
    regen_power = regen_power[:stop_idx + 1]

    t_stop = t[-1]
    d_stop = d[-1]
    W_total = 0.5 * mass_kg * v0 ** 2
    W_regen = np.trapezoid(regen_power, t) * REGEN_EFFICIENCY  # Joules actually banked
    regen_kwh = W_regen / 3.6e6
    co2_saved = regen_kwh * CO2_PER_KWH
    homes_days = regen_kwh / HOME_KWH_PER_DAY

    return dict(t=t, v=v, v_kmh=v * 3.6, d=d, F_motor=F_motor, slip=slip_arr,
                regen_power_kw=regen_power / 1000.0, t_stop=t_stop, d_stop=d_stop,
                W_total=W_total, regen_kwh=regen_kwh, co2_saved=co2_saved,
                homes_days=homes_days, t_stop_conv=t_stop_conv,
                d_stop_conv=v0 ** 2 / (2 * conv_decel))


# ---------------------------------------------------------------
# GUI
# ---------------------------------------------------------------
def main():
    plt.rcParams["font.size"] = 9
    fig = plt.figure(figsize=(14, 8.5), facecolor="#0d1117")
    fig.canvas.manager.set_window_title("Induction Motor Regenerative Braking — Analysis")

    gs = GridSpec(3, 3, figure=fig, left=0.07, right=0.97, top=0.90, bottom=0.22,
                  hspace=0.55, wspace=0.32)

    ax_torque = fig.add_subplot(gs[0:2, 0])       # torque-slip curve, big
    ax_vt = fig.add_subplot(gs[0, 1:])            # v-t comparison
    ax_power = fig.add_subplot(gs[1, 1])          # regen power vs time
    ax_stats = fig.add_subplot(gs[1, 2])          # stats panel
    ax_bar = fig.add_subplot(gs[2, :])            # energy bar

    for ax in [ax_torque, ax_vt, ax_power, ax_bar]:
        ax.set_facecolor("#161b22")
        ax.tick_params(colors="#c9d1d9")
        for spine in ax.spines.values():
            spine.set_color("#30363d")
        ax.title.set_color("white")
        ax.xaxis.label.set_color("#c9d1d9")
        ax.yaxis.label.set_color("#c9d1d9")
    ax_stats.axis("off")

    fig.suptitle("Phase Induction Motor — Regenerative Braking Analysis",
                 color="white", fontsize=15, fontweight="bold", y=0.965)

    state = {"preset": "INDIAN", "target_slip": -0.12}

    def redraw(_=None):
        p = PRESETS[state["preset"]]
        ts = state["target_slip"]
        run = build_run(p["speed"], p["mass"], p["decel"], p["t_max"], ts)

        # --- Torque-slip curve ---
        ax_torque.clear()
        ax_torque.set_facecolor("#161b22")
        s_curve = np.linspace(-0.9, 0.9, 400)
        T_curve = kloss_torque(s_curve, p["t_max"], SLIP_MAX_TORQUE)
        ax_torque.plot(s_curve, T_curve, color="#58a6ff", linewidth=2)
        ax_torque.axhline(0, color="#8b949e", linewidth=0.8)
        ax_torque.axvline(0, color="#8b949e", linewidth=0.8, linestyle="--")
        ax_torque.axvspan(-0.9, 0, color="#3fb950", alpha=0.08)
        ax_torque.axvspan(0, 0.9, color="#f0883e", alpha=0.06)
        ax_torque.text(-0.7, T_curve.max() * 0.85, "REGEN\n(braking +\nenergy back)",
                       color="#3fb950", fontsize=8, fontweight="bold", ha="center")
        ax_torque.text(0.5, T_curve.max() * 0.85, "MOTORING\n(driving)",
                       color="#f0883e", fontsize=8, fontweight="bold", ha="center")
        cur_T = kloss_torque(np.array([ts]), p["t_max"], SLIP_MAX_TORQUE)[0]
        ax_torque.plot([ts], [cur_T], "o", color="white", markersize=10, zorder=5)
        ax_torque.plot([ts], [cur_T], "o", color="#3fb950", markersize=6, zorder=6)
        ax_torque.set_xlabel("Slip  s")
        ax_torque.set_ylabel("Torque (N·m)")
        ax_torque.set_title("Torque–Slip Curve (Kloss)\nwhite dot = operating point", fontsize=10)
        ax_torque.grid(alpha=0.15)

        # --- v-t comparison ---
        ax_vt.clear()
        ax_vt.set_facecolor("#161b22")
        v0 = p["speed"] / 3.6
        t_conv = np.linspace(0, run["t_stop_conv"], 200)
        v_conv = np.maximum(0, v0 - p["decel"] * t_conv) * 3.6
        ax_vt.plot(t_conv, v_conv, "--", color="#8b949e", linewidth=1.8, label="Conventional only")
        ax_vt.plot(run["t"], run["v_kmh"], color="#3fb950", linewidth=2.2, label="+ Motor regen braking")
        ax_vt.fill_between(run["t"], 0, run["v_kmh"], color="#3fb950", alpha=0.08)
        ax_vt.set_xlabel("Time (s)")
        ax_vt.set_ylabel("Speed (km/h)")
        ax_vt.set_title(f"Stopping time: {run['t_stop_conv']:.1f}s → {run['t_stop']:.1f}s   "
                        f"({run['t_stop_conv']-run['t_stop']:.1f}s sooner)", fontsize=10)
        ax_vt.legend(facecolor="#21262d", labelcolor="white", fontsize=8, loc="upper right")
        ax_vt.grid(alpha=0.15)

        # --- Regen power vs time ---
        ax_power.clear()
        ax_power.set_facecolor("#161b22")
        ax_power.plot(run["t"], run["regen_power_kw"], color="#d29922", linewidth=2)
        ax_power.fill_between(run["t"], 0, run["regen_power_kw"], color="#d29922", alpha=0.2)
        ax_power.set_xlabel("Time (s)")
        ax_power.set_ylabel("Power (kW)")
        ax_power.set_title("Power fed back to catenary", fontsize=10)
        ax_power.grid(alpha=0.15)

        # --- Stats panel ---
        ax_stats.clear()
        ax_stats.axis("off")
        stats_text = (
            f"Preset: {state['preset']}\n"
            f"Target slip: {ts:.2f}\n\n"
            f"Stop distance:\n"
            f"  {run['d_stop_conv']:.0f} m → {run['d_stop']:.0f} m\n"
            f"  ({run['d_stop_conv']-run['d_stop']:.0f} m shorter)\n\n"
            f"Regenerated: {run['regen_kwh']:.2f} kWh\n"
            f"CO2 saved: {run['co2_saved']:.2f} kg\n"
            f"Homes powered: {run['homes_days']:.2f} home-days"
        )
        ax_stats.text(0.02, 0.95, stats_text, transform=ax_stats.transAxes,
                      color="white", fontsize=9, va="top", family="monospace",
                      linespacing=1.6)

        # --- Energy bar ---
        ax_bar.clear()
        ax_bar.set_facecolor("#161b22")
        W_total_kwh = run["W_total"] / 3.6e6
        W_regen = run["regen_kwh"]
        W_other = max(0.0, W_total_kwh - W_regen)
        ax_bar.barh(["Energy split"], [W_regen], color="#3fb950", label=f"Regenerated ({W_regen:.2f} kWh)")
        ax_bar.barh(["Energy split"], [W_other], left=[W_regen], color="#484f58",
                    label=f"Friction heat / losses ({W_other:.2f} kWh)")
        ax_bar.set_xlabel("Total kinetic energy (kWh)")
        ax_bar.set_title(f"Total KE this stop: {W_total_kwh:.2f} kWh", fontsize=10)
        ax_bar.legend(facecolor="#21262d", labelcolor="white", fontsize=8, loc="lower right")
        ax_bar.grid(alpha=0.15, axis="x")

        fig.canvas.draw_idle()

    # --- Widgets ---
    ax_slider = fig.add_axes([0.15, 0.10, 0.55, 0.03])
    ax_slider.set_facecolor("#21262d")
    slip_slider = Slider(ax_slider, "Target slip (regen depth)", -0.30, -0.02,
                         valinit=state["target_slip"], valstep=0.01,
                         color="#3fb950")
    slip_slider.label.set_color("white")
    slip_slider.valtext.set_color("white")

    def on_slip_change(val):
        state["target_slip"] = val
        redraw()
    slip_slider.on_changed(on_slip_change)

    ax_indian = fig.add_axes([0.75, 0.06, 0.1, 0.05])
    ax_bullet = fig.add_axes([0.86, 0.06, 0.1, 0.05])
    btn_indian = Button(ax_indian, "INDIAN", color="#1f6feb", hovercolor="#388bfd")
    btn_bullet = Button(ax_bullet, "BULLET", color="#8b1a1a", hovercolor="#b62324")

    def set_preset(name):
        def _handler(event):
            state["preset"] = name
            redraw()
        return _handler
    btn_indian.on_clicked(set_preset("INDIAN"))
    btn_bullet.on_clicked(set_preset("BULLET"))

    fig.text(0.15, 0.155, "Drag the slider to command a deeper regen slip (more negative = "
                          "stronger braking torque, up to breakdown).",
             color="#8b949e", fontsize=8.5)

    redraw()
    plt.show()


if __name__ == "__main__":
    main()
