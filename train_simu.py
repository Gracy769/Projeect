
import math
import sys
import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Button, TextBox, Slider


# ─────────────────────────────────────────────────────────────────────────────
# Constants & presets
# ─────────────────────────────────────────────────────────────────────────────
PRESETS = {
    "INDIAN": {
        "speed": 160.0, "mass": 400.0, "decel": 0.8,
        "body_color": "#1f6feb", "edge_color": "#388bfd",
        "cab_color":  "#0c447c", "win_color":  "#79c0ff",
        "label": "Indian Railway",
    },
    "BULLET": {
        "speed": 200.0, "mass": 400.0, "decel": 1.0,
        "body_color": "#8b1a1a", "edge_color": "#f78166",
        "cab_color":  "#5c0d0d", "win_color":  "#ffb3b3",
        "label": "Bullet Train",
    },
}

REGEN_EFFICIENCY  = 0.50   # 50 % of dissipated energy recovered
CO2_PER_KWH       = 0.233  # kg CO2 per kWh (India grid average)
HOME_KWH_PER_DAY  = 8


# ─────────────────────────────────────────────────────────────────────────────
# Physics helpers
# ─────────────────────────────────────────────────────────────────────────────
def compute_physics(speed_kmh, decel, mass_t):
    v0      = speed_kmh / 3.6
    mass_kg = mass_t * 1000.0
    t_stop  = v0 / decel
    d_stop  = (v0 ** 2) / (2.0 * decel)
    W       = 0.5 * mass_kg * v0 ** 2
    return v0, mass_kg, t_stop, d_stop, W


def build_time_series(v0, decel, t_stop, n=300):
    t     = np.linspace(0, t_stop, n)
    v     = np.maximum(0.0, v0 - decel * t)
    v_kmh = v * 3.6
    d     = np.clip(v0 * t - 0.5 * decel * t ** 2, 0.0, v0 ** 2 / (2.0 * decel))
    return t, v_kmh, d


def fmt_energy(W):
    if W >= 1e9:  return f"{W/1e9:.3f} GJ"
    if W >= 1e6:  return f"{W/1e6:.2f} MJ"
    return              f"{W/1e3:.1f} kJ"


def fmt_distance(d):
    if d >= 1000: return f"{d/1000:.3f} km"
    return              f"{d:.1f} m"


def print_results(speed_kmh, decel, mass_t, v0, mass_kg, t_stop, d_stop, W):
    sep = "=" * 52
    print(f"\n{sep}")
    print("              SIMULATION RESULTS")
    print(sep)
    print(f"  Initial speed      : {speed_kmh:.1f} km/h  ({v0:.2f} m/s)")
    print(f"  Deceleration       : {decel:.3f} m/s2")
    print(f"  Train mass         : {mass_t:.1f} t  ({mass_kg:,.0f} kg)")
    print()
    print(f"  Time to stop       : {t_stop:.2f} s")
    print(f"  Stopping distance  : {fmt_distance(d_stop)}")
    print(f"  Work done (braking): {fmt_energy(W)}  ({W:.3e} J)")
    print(sep)
    print("  Physics:  t = v0/a   |   d = v0^2/(2a)   |   W = 0.5*m*v0^2")
    print(sep + "\n")


# ─────────────────────────────────────────────────────────────────────────────
# CLI input
# ─────────────────────────────────────────────────────────────────────────────
def get_inputs():
    print("=" * 52)
    print("         TRAIN BRAKING SIMULATION  v3")
    print("=" * 52)
    print()
    print("  GUI presets:  INDIAN (160 km/h, 400 t, 0.8 m/s2)")
    print("                BULLET (200 km/h, 400 t, 3.8 m/s2)")
    print()

    def ask(prompt, lo=0.0):
        while True:
            try:
                val = float(input(prompt))
                if val <= lo:
                    print(f"  Value must be > {lo}.")
                    continue
                return val
            except ValueError:
                print("  Please enter a valid number.")

    s = ask("Enter initial train speed (km/h): ")
    a = ask("Enter braking deceleration (m/s2): ")
    m = ask("Enter train mass (tonnes): ")
    return s, a, m


# ─────────────────────────────────────────────────────────────────────────────
# Eddy-current braking window
# ─────────────────────────────────────────────────────────────────────────────
def open_eddy_window(speed_kmh, mass_t, conv_decel):
    """Open a second figure showing eddy-current braking analysis."""

    v0      = speed_kmh / 3.6
    mass_kg = mass_t * 1000.0
    W_total = 0.5 * mass_kg * v0 ** 2   # total kinetic energy

    fig2 = plt.figure(figsize=(13, 8), facecolor="#0d1117")
    fig2.suptitle("Eddy Current Braking Analysis", fontsize=15,
                  color="white", fontweight="bold", y=0.97)

    gs2 = GridSpec(2, 3, figure=fig2, hspace=0.55, wspace=0.42,
                   top=0.89, bottom=0.18, left=0.08, right=0.97)

    ax_vt   = fig2.add_subplot(gs2[0, :2])   # v-t comparison
    ax_bar  = fig2.add_subplot(gs2[1, :2])   # energy bar chart
    ax_stat = fig2.add_subplot(gs2[:, 2])    # stats panel

    for ax in [ax_vt, ax_bar, ax_stat]:
        ax.set_facecolor("#161b22")
        for sp in ax.spines.values():
            sp.set_edgecolor("#30363d")
    ax_stat.axis("off")

    # ── slider ────────────────────────────────────────────────────────────────
    ax_sld = fig2.add_axes([0.15, 0.07, 0.55, 0.04])
    ax_sld.set_facecolor("#161b22")
    for sp in ax_sld.spines.values():
        sp.set_edgecolor("#30363d")

    # eddy decel starts at 30 % extra above conventional
    eddy_init = round(conv_decel * 0.30, 2)
    sld = Slider(ax_sld, "Eddy decel add-on (m/s2)",
                 valmin=0.0, valmax=max(conv_decel * 2.0, 5.0),
                 valinit=eddy_init, valstep=0.05,
                 color="#1f6feb", initcolor="#388bfd")
    sld.label.set_color("white")
    sld.label.set_fontsize(9)
    sld.valtext.set_color("#58a6ff")
    sld.valtext.set_fontsize(9)

    fig2.text(0.15, 0.125,
              "Eddy current braking adds a contactless, wear-free deceleration component to the\n"
              "conventional friction brakes. Drag the slider to change its contribution.",
              color="#8b949e", fontsize=8, va="top")

    # ── draw function (called on slider change) ────────────────────────────────
    def redraw(eddy_addon):
        total_decel = conv_decel + eddy_addon

        # time series
        t_conv,  v_conv,  d_conv  = build_time_series(v0, conv_decel,  v0 / conv_decel)
        t_eddy,  v_eddy,  d_eddy  = build_time_series(v0, total_decel, v0 / total_decel)

        t_stop_conv  = v0 / conv_decel
        t_stop_eddy  = v0 / total_decel
        d_stop_conv  = (v0 ** 2) / (2.0 * conv_decel)
        d_stop_eddy  = (v0 ** 2) / (2.0 * total_decel)

        # energy split
        # Eddy current braking is the part that can be regenerated
        W_eddy_frac  = eddy_addon / total_decel if total_decel > 0 else 0.0
        W_dissipated = W_total * (1.0 - W_eddy_frac)   # heat in friction brakes
        W_eddy_raw   = W_total * W_eddy_frac            # captured by eddy / regen
        W_regen      = W_eddy_raw * REGEN_EFFICIENCY    # actually recovered
        W_lost_eddy  = W_eddy_raw * (1.0 - REGEN_EFFICIENCY)

        regen_kwh    = W_regen / 3.6e6
        co2_saved    = regen_kwh * CO2_PER_KWH
        homes_days   = regen_kwh / HOME_KWH_PER_DAY

        # ── v-t chart ─────────────────────────────────────────────────────────
        ax_vt.cla()
        ax_vt.set_facecolor("#161b22")
        for sp in ax_vt.spines.values():
            sp.set_edgecolor("#30363d")
        ax_vt.set_title("Velocity – Time Comparison", color="white", fontsize=11, pad=6)
        ax_vt.plot(t_conv, v_conv, color="#58a6ff", linewidth=2,
                   label=f"Conventional  (a={conv_decel:.2f} m/s2,  stop in {t_stop_conv:.1f}s)")
        ax_vt.plot(t_eddy, v_eddy, color="#3fb950", linewidth=2, linestyle="--",
                   label=f"+ Eddy current (a={total_decel:.2f} m/s2,  stop in {t_stop_eddy:.1f}s)")
        ax_vt.fill_between(t_conv, v_conv, alpha=0.10, color="#58a6ff")
        ax_vt.fill_between(t_eddy, v_eddy, alpha=0.10, color="#3fb950")
        ax_vt.set_xlabel("Time (s)",     color="#8b949e", fontsize=9)
        ax_vt.set_ylabel("Speed (km/h)", color="#8b949e", fontsize=9)
        ax_vt.tick_params(colors="#8b949e", labelsize=8)
        ax_vt.set_xlim(0, t_stop_conv * 1.05)
        ax_vt.set_ylim(0, speed_kmh * 1.1)
        ax_vt.grid(True, color="#21262d", linewidth=0.5)
        leg = ax_vt.legend(fontsize=8, facecolor="#161b22", edgecolor="#30363d",
                           labelcolor="white", loc="upper right")

        # time saved annotation
        dt = t_stop_conv - t_stop_eddy
        ax_vt.annotate(f"  {dt:.1f}s saved",
                       xy=(t_stop_eddy, 0), xytext=(t_stop_eddy + t_stop_conv * 0.05, speed_kmh * 0.12),
                       color="#f0883e", fontsize=8,
                       arrowprops=dict(arrowstyle="->", color="#f0883e", lw=1.2))

        # ── energy bar chart ──────────────────────────────────────────────────
        ax_bar.cla()
        ax_bar.set_facecolor("#161b22")
        for sp in ax_bar.spines.values():
            sp.set_edgecolor("#30363d")
        ax_bar.set_title("Energy Distribution", color="white", fontsize=11, pad=6)

        categories  = ["Conventional\nBraking", "With Eddy\nCurrent"]
        heat_vals   = [W_total / 1e6,  W_dissipated / 1e6]
        eddy_vals   = [0.0,            W_lost_eddy  / 1e6]
        regen_vals  = [0.0,            W_regen      / 1e6]

        x = np.arange(len(categories))
        bw = 0.45
        bars_heat  = ax_bar.bar(x, heat_vals,  bw, label="Heat (friction)", color="#f78166", alpha=0.85)
        bars_eddy  = ax_bar.bar(x, eddy_vals,  bw, bottom=heat_vals,
                                label="Eddy losses (non-regen)", color="#d29922", alpha=0.85)
        bars_regen = ax_bar.bar(x, regen_vals, bw,
                                bottom=[h + e for h, e in zip(heat_vals, eddy_vals)],
                                label="Regenerated energy", color="#3fb950", alpha=0.90)

        ax_bar.set_xticks(x)
        ax_bar.set_xticklabels(categories, color="white", fontsize=9)
        ax_bar.set_ylabel("Energy (MJ)", color="#8b949e", fontsize=9)
        ax_bar.tick_params(colors="#8b949e", labelsize=8)
        ax_bar.set_ylim(0, W_total / 1e6 * 1.25)
        ax_bar.grid(True, axis="y", color="#21262d", linewidth=0.5)
        ax_bar.legend(fontsize=8, facecolor="#161b22", edgecolor="#30363d",
                      labelcolor="white", loc="upper right")

        # value labels on bars
        for bar in bars_regen:
            h = bar.get_height()
            if h > W_total / 1e6 * 0.02:
                ax_bar.text(bar.get_x() + bar.get_width() / 2,
                            bar.get_y() + h / 2,
                            f"{h:.1f}", ha="center", va="center",
                            color="white", fontsize=8, fontweight="bold")

        # ── stats panel ───────────────────────────────────────────────────────
        ax_stat.cla()
        ax_stat.axis("off")
        ax_stat.set_xlim(0, 1)
        ax_stat.set_ylim(0, 1)

        rows = [
            ("SCENARIO",                 "#8b949e", 13, 0.97),
            (f"{speed_kmh:.0f} km/h  |  {mass_t:.0f} t", "white", 10, 0.91),
            ("",                         "#8b949e", 10, 0.85),

            ("CONVENTIONAL",             "#58a6ff", 12, 0.82),
            (f"Decel: {conv_decel:.2f} m/s2",     "white", 10, 0.76),
            (f"Stop time: {t_stop_conv:.1f} s",    "white", 10, 0.70),
            (f"Distance: {fmt_distance(d_stop_conv)}", "white", 10, 0.64),
            ("",                         "#8b949e", 10, 0.58),

            ("WITH EDDY CURRENT",        "#3fb950", 12, 0.55),
            (f"Decel: {total_decel:.2f} m/s2",     "white", 10, 0.49),
            (f"Stop time: {t_stop_eddy:.1f} s",    "white", 10, 0.43),
            (f"Distance: {fmt_distance(d_stop_eddy)}", "white", 10, 0.37),
            ("",                         "#8b949e", 10, 0.31),

            ("REGENERATED",              "#f0883e", 12, 0.28),
            (fmt_energy(W_regen),        "#3fb950", 14, 0.21),
            (f"{regen_kwh:.2f} kWh",    "white",   10, 0.15),
            (f"CO2 saved: {co2_saved:.2f} kg",  "#3fb950", 9, 0.09),
        ]
        for txt, col, sz, yp in rows:
            ax_stat.text(0.05, yp, txt, color=col, fontsize=sz,
                         fontweight="bold" if sz >= 12 else "normal",
                         transform=ax_stat.transAxes, va="center")

        fig2.canvas.draw_idle()

    # initial draw
    redraw(eddy_init)

    # connect slider
    def on_slider(val):
        redraw(val)

    sld.on_changed(on_slider)

    plt.show()
    return sld   # keep reference


# ─────────────────────────────────────────────────────────────────────────────
# Main simulation window
# ─────────────────────────────────────────────────────────────────────────────
def run_simulation(speed_kmh, decel, mass_t):
    v0, mass_kg, t_stop, d_stop, W = compute_physics(speed_kmh, decel, mass_t)
    print_results(speed_kmh, decel, mass_t, v0, mass_kg, t_stop, d_stop, W)
    t, v_kmh, d = build_time_series(v0, decel, t_stop)

    # default train colours (custom entry – blue)
    train_colors = {
        "body": "#1f6feb", "edge": "#388bfd",
        "cab":  "#0c447c", "win":  "#79c0ff",
    }

    # ── figure ────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 10), facecolor="#0d1117")
    fig.suptitle("Train Braking Simulation", fontsize=16, color="white",
                 fontweight="bold", y=0.98)

    gs = GridSpec(3, 3, figure=fig, hspace=0.55, wspace=0.4,
                  top=0.92, bottom=0.14, left=0.07, right=0.97)

    ax_track = fig.add_subplot(gs[0, :])
    ax_vel   = fig.add_subplot(gs[1, :2])
    ax_dist  = fig.add_subplot(gs[2, :2])
    ax_info  = fig.add_subplot(gs[1:, 2])

    for ax in [ax_track, ax_vel, ax_dist, ax_info]:
        ax.set_facecolor("#161b22")
        for sp in ax.spines.values():
            sp.set_edgecolor("#30363d")

    # ── shared state ──────────────────────────────────────────────────────────
    state   = {"speed_kmh": speed_kmh, "decel": decel, "mass_t": mass_t,
               "anim": None, "anim_running": False}
    ts_ref  = [t, v_kmh, d]
    vp_ref  = [None]
    dp_ref  = [None]
    sld_ref = [None]   # keep eddy slider alive

    # ── info panel ────────────────────────────────────────────────────────────
    def draw_info(s, a, m, ts, ds, work):
        ax_info.cla()
        ax_info.axis("off")
        ax_info.set_xlim(0, 1); ax_info.set_ylim(0, 1)
        rows = [
            ("INPUTS",              "#8b949e", 13, 0.95),
            (f"Speed : {s:.0f} km/h", "white", 10, 0.88),
            (f"Decel : {a:.3f} m/s2", "white", 10, 0.81),
            (f"Mass  : {m:.0f} t",    "white", 10, 0.74),
            ("",                    "#8b949e", 10, 0.67),
            ("RESULTS",             "#8b949e", 13, 0.63),
            ("Stop time",           "#8b949e",  9, 0.56),
            (f"{ts:.2f} s",        "#58a6ff", 14, 0.49),
            ("Distance",            "#8b949e",  9, 0.41),
            (fmt_distance(ds),     "#3fb950", 14, 0.34),
            ("Work done",           "#8b949e",  9, 0.25),
            (fmt_energy(work),     "#f78166", 14, 0.18),
        ]
        for txt, col, sz, yp in rows:
            ax_info.text(0.08, yp, txt, color=col, fontsize=sz,
                         fontweight="bold" if sz >= 13 else "normal",
                         transform=ax_info.transAxes, va="center")

    draw_info(speed_kmh, decel, mass_t, t_stop, d_stop, W)

    # ── chart helpers ─────────────────────────────────────────────────────────
    def setup_vel_chart(tn, vcn, s, ts):
        ax_vel.cla()
        ax_vel.set_facecolor("#161b22")
        for sp in ax_vel.spines.values(): sp.set_edgecolor("#30363d")
        ax_vel.set_title("Velocity – Time", color="white", fontsize=11, pad=6)
        ax_vel.plot(tn, vcn, color="#58a6ff", linewidth=2)
        ax_vel.fill_between(tn, vcn, alpha=0.15, color="#58a6ff")
        ax_vel.set_xlabel("Time (s)",     color="#8b949e", fontsize=9)
        ax_vel.set_ylabel("Speed (km/h)", color="#8b949e", fontsize=9)
        ax_vel.tick_params(colors="#8b949e", labelsize=8)
        ax_vel.set_xlim(0, ts); ax_vel.set_ylim(0, s * 1.1)
        ax_vel.grid(True, color="#21262d", linewidth=0.5)
        vp, = ax_vel.plot([], [], 'o', color="#f0883e", markersize=7, zorder=5)
        return vp

    def setup_dist_chart(tn, dn, ds, ts):
        ax_dist.cla()
        ax_dist.set_facecolor("#161b22")
        for sp in ax_dist.spines.values(): sp.set_edgecolor("#30363d")
        ax_dist.set_title("Distance – Time", color="white", fontsize=11, pad=6)
        ax_dist.plot(tn, dn, color="#3fb950", linewidth=2)
        ax_dist.fill_between(tn, dn, alpha=0.15, color="#3fb950")
        ax_dist.set_xlabel("Time (s)",     color="#8b949e", fontsize=9)
        ax_dist.set_ylabel("Distance (m)", color="#8b949e", fontsize=9)
        ax_dist.tick_params(colors="#8b949e", labelsize=8)
        ax_dist.set_xlim(0, ts); ax_dist.set_ylim(0, ds * 1.1)
        ax_dist.grid(True, color="#21262d", linewidth=0.5)
        dp, = ax_dist.plot([], [], 'o', color="#f0883e", markersize=7, zorder=5)
        return dp

    vp_ref[0] = setup_vel_chart(t, v_kmh, speed_kmh, t_stop)
    dp_ref[0] = setup_dist_chart(t, d, d_stop, t_stop)

    # ── track view ────────────────────────────────────────────────────────────
    ax_track.set_title("Track View", color="white", fontsize=11, pad=6)
    ax_track.set_xlim(0, 100); ax_track.set_ylim(0, 10)
    ax_track.axis("off")

    ax_track.add_patch(patches.FancyArrowPatch(
        (0, 3.5), (100, 3.5), arrowstyle="-", color="#484f58", linewidth=3))
    ax_track.add_patch(patches.FancyArrowPatch(
        (0, 2.5), (100, 2.5), arrowstyle="-", color="#484f58", linewidth=3))
    for x in np.arange(1, 100, 4):
        ax_track.add_patch(patches.Rectangle(
            (x, 2.0), 1.2, 2.0, color="#30363d", zorder=1))

    speed_label = ax_track.text(50, 8.5, f"{speed_kmh:.0f} km/h",
                                 color="#f0883e", fontsize=12,
                                 ha="center", fontweight="bold")
    phase_label = ax_track.text(50, 7.2, "CRUISING",
                                 color="#3fb950", fontsize=9, ha="center")

    TRAIN_W, TRAIN_H = 14, 3.5

    train_body = patches.FancyBboxPatch(
        (5, 3.5), TRAIN_W, TRAIN_H, boxstyle="round,pad=0.3",
        facecolor=train_colors["body"], edgecolor=train_colors["edge"],
        linewidth=1.5, zorder=4)
    ax_track.add_patch(train_body)

    cab = patches.Polygon(
        [[5+TRAIN_W, 3.5], [5+TRAIN_W+2.5, 5.3], [5+TRAIN_W, 3.5+TRAIN_H]],
        closed=True, facecolor=train_colors["cab"], zorder=4)
    ax_track.add_patch(cab)

    win_patches = []
    for wx in [6.5, 10, 13.5]:
        w = patches.FancyBboxPatch((wx, 5.5), 2, 1.2, boxstyle="round,pad=0.1",
                                    facecolor=train_colors["win"],
                                    alpha=0.7, edgecolor="none", zorder=5)
        ax_track.add_patch(w)
        win_patches.append(w)

    wheel_patches = []
    for wx in [7, 12, 17]:
        wh = plt.Circle((wx, 3.2), 0.9, color="#161b22",
                         zorder=5, linewidth=1.5, ec="#484f58")
        ax_track.add_patch(wh)
        wheel_patches.append(wh)

    spark_patches = []
    for wx in [7, 12, 17]:
        for dy in [-0.3, 0.3]:
            sp = plt.Circle((wx, 3.0+dy), 0.25,
                             color="#f0883e", alpha=0, zorder=6)
            ax_track.add_patch(sp)
            spark_patches.append(sp)

    brake_bg   = patches.Rectangle((5, 0.4), TRAIN_W+2.5, 0.8,
                                    color="#21262d", zorder=3)
    brake_fill = patches.Rectangle((5, 0.4), 0, 0.8,
                                    color="#f78166", zorder=4)
    ax_track.add_patch(brake_bg)
    ax_track.add_patch(brake_fill)
    ax_track.text(4.8, 0.8, "Brake", color="#8b949e",
                  fontsize=7, ha="right", va="center")

    # ── animation ─────────────────────────────────────────────────────────────
    N_FRAMES      = 180
    CRUISE_FRAC   = 0.25
    cruise_frames = int(N_FRAMES * CRUISE_FRAC)
    decel_frames  = N_FRAMES - cruise_frames

    def animate_frame(frame):
        t_cur, vc_cur, d_cur = ts_ref
        vp = vp_ref[0];  dp = dp_ref[0]
        spd = state["speed_kmh"]

        if frame < cruise_frames:
            prog  = frame / cruise_frames
            x_pos = 5 + prog * 20
            cur_v = spd;  brake_frac = 0.0
            phase_label.set_text("CRUISING"); phase_label.set_color("#3fb950")
            for sp in spark_patches: sp.set_alpha(0)
        else:
            b_prog = (frame - cruise_frames) / decel_frames
            x_pos  = 25 + b_prog * 55
            idx    = min(int(b_prog * len(t_cur)), len(t_cur) - 1)
            cur_v  = vc_cur[idx];  brake_frac = b_prog
            if vp is not None: vp.set_data([t_cur[idx]], [vc_cur[idx]])
            if dp is not None: dp.set_data([t_cur[idx]], [d_cur[idx]])
            ph = "BRAKING" if cur_v > 0.5 else "STOPPED"
            phase_label.set_text(ph)
            phase_label.set_color("#f0883e" if cur_v > 0.5 else "#8b949e")
            sa = 0.8 if cur_v > 2 else 0
            for sp in spark_patches:
                sp.set_alpha(sa * (0.7 + 0.3 * math.sin(frame * 1.5)))

        delta = x_pos - 5
        train_body.set_x(5 + delta)
        cab.set_xy([[5+TRAIN_W+delta, 3.5],
                    [5+TRAIN_W+delta+2.5, 5.3],
                    [5+TRAIN_W+delta, 3.5+TRAIN_H]])
        for i, wx in enumerate([7, 12, 17]):
            win_patches[i].set_x(wx + delta - 1)
            wheel_patches[i].center = (wx + delta, 3.2)
        for j, (wx, dy) in enumerate([(7,-0.3),(7,0.3),(12,-0.3),(12,0.3),(17,-0.3),(17,0.3)]):
            spark_patches[j].center = (wx + delta, 3.0 + dy)

        speed_label.set_text(f"{cur_v:.1f} km/h")
        brake_fill.set_width((TRAIN_W + 2.5) * brake_frac)

        artists = ([train_body, cab, speed_label, phase_label, brake_fill]
                   + win_patches + wheel_patches + spark_patches)
        if vp is not None: artists.append(vp)
        if dp is not None: artists.append(dp)
        return artists

    def stop_anim():
        """Safely stop any running animation."""
        if state["anim"] is not None:
            try:
                state["anim"].event_source.stop()
            except Exception:
                pass
            state["anim"] = None
        state["anim_running"] = False

    def reset_train():
        train_body.set_x(5)
        cab.set_xy([[5+TRAIN_W, 3.5], [5+TRAIN_W+2.5, 5.3], [5+TRAIN_W, 3.5+TRAIN_H]])
        for i, wx in enumerate([7, 12, 17]):
            win_patches[i].set_x(wx - 1)
            wheel_patches[i].center = (wx, 3.2)
        for j, (wx, dy) in enumerate([(7,-0.3),(7,0.3),(12,-0.3),(12,0.3),(17,-0.3),(17,0.3)]):
            spark_patches[j].center = (wx, 3.0 + dy)
        brake_fill.set_width(0)
        if vp_ref[0] is not None: vp_ref[0].set_data([], [])
        if dp_ref[0] is not None: dp_ref[0].set_data([], [])
        phase_label.set_text("CRUISING"); phase_label.set_color("#3fb950")
        speed_label.set_text(f"{state['speed_kmh']:.0f} km/h")

    def start_anim():
        stop_anim()
        reset_train()
        state["anim"] = animation.FuncAnimation(
            fig, animate_frame, frames=N_FRAMES,
            interval=40, blit=True, repeat=False)
        state["anim_running"] = True
        fig.canvas.draw_idle()

    def recolor_train(colors):
        train_body.set_facecolor(colors["body"])
        train_body.set_edgecolor(colors["edge"])
        cab.set_facecolor(colors["cab"])
        for w in win_patches:
            w.set_facecolor(colors["win"])

    start_anim()

    # ── apply values (preset or custom) ───────────────────────────────────────
    def apply_values(s, a, m, colors=None):
        state["speed_kmh"] = s
        state["decel"]     = a
        state["mass_t"]    = m

        v0n, mkgn, tsn, dsn, Wn = compute_physics(s, a, m)
        print_results(s, a, m, v0n, mkgn, tsn, dsn, Wn)

        tn, vcn, dn = build_time_series(v0n, a, tsn)
        ts_ref[0], ts_ref[1], ts_ref[2] = tn, vcn, dn

        vp_ref[0] = setup_vel_chart(tn, vcn, s, tsn)
        dp_ref[0] = setup_dist_chart(tn, dn, dsn, tsn)
        draw_info(s, a, m, tsn, dsn, Wn)

        if colors:
            recolor_train(colors)

        start_anim()

    # ── widget axes ───────────────────────────────────────────────────────────
    bh = 0.050;  by = 0.032

    ax_bi  = fig.add_axes([0.07,  by, 0.09, bh])
    ax_bb  = fig.add_axes([0.175, by, 0.09, bh])
    ax_ts  = fig.add_axes([0.33,  by, 0.09, bh])
    ax_td  = fig.add_axes([0.44,  by, 0.09, bh])
    ax_tm  = fig.add_axes([0.55,  by, 0.09, bh])
    ax_br  = fig.add_axes([0.67,  by, 0.08, bh])
    ax_be  = fig.add_axes([0.77,  by, 0.15, bh])   # eddy window button

    for ax in [ax_bi, ax_bb, ax_ts, ax_td, ax_tm, ax_br, ax_be]:
        ax.set_facecolor("#161b22")
        for sp in ax.spines.values(): sp.set_edgecolor("#30363d")

    # labels above text boxes
    for xc, lbl in [(0.375, "Speed\n(km/h)"), (0.485, "Decel\n(m/s2)"), (0.595, "Mass\n(t)")]:
        fig.text(xc, by + bh + 0.003, lbl, color="#8b949e", fontsize=6.5, ha="center")

    btn_indian = Button(ax_bi,  "INDIAN", color="#0d2a4a", hovercolor="#1f6feb")
    btn_bullet = Button(ax_bb,  "BULLET", color="#2a0d0d", hovercolor="#8b1a1a")
    btn_run    = Button(ax_br,  "RUN",    color="#0d2a1a", hovercolor="#238636")
    btn_eddy   = Button(ax_be,  "Eddy Current Analysis >>>",
                        color="#1a1a2e", hovercolor="#2d2d5e")

    btn_indian.label.set_color("#58a6ff"); btn_indian.label.set_fontsize(9);  btn_indian.label.set_fontweight("bold")
    btn_bullet.label.set_color("#f78166"); btn_bullet.label.set_fontsize(9);  btn_bullet.label.set_fontweight("bold")
    btn_run.label.set_color("#3fb950");    btn_run.label.set_fontsize(9);     btn_run.label.set_fontweight("bold")
    btn_eddy.label.set_color("#c9a1ff");   btn_eddy.label.set_fontsize(8.5);  btn_eddy.label.set_fontweight("bold")

    tb_speed = TextBox(ax_ts, "", initial=str(int(speed_kmh)), color="#161b22", hovercolor="#21262d")
    tb_decel = TextBox(ax_td, "", initial=str(decel),          color="#161b22", hovercolor="#21262d")
    tb_mass  = TextBox(ax_tm, "", initial=str(int(mass_t)),    color="#161b22", hovercolor="#21262d")

    for tb in [tb_speed, tb_decel, tb_mass]:
        tb.text_disp.set_color("white"); tb.text_disp.set_fontsize(9)

    fig.text(0.12,  by + bh + 0.003, "Presets",       color="#8b949e", fontsize=6.5, ha="center")
    fig.text(0.30,  by + bh * 0.5,   "|",             color="#30363d", fontsize=16,  va="center")
    fig.text(0.49,  by - 0.016,       "Custom values", color="#8b949e", fontsize=6.5, ha="center")

    # ── callbacks ─────────────────────────────────────────────────────────────
    def on_indian(_e):
        p = PRESETS["INDIAN"]
        tb_speed.set_val(str(int(p["speed"])))
        tb_decel.set_val(str(p["decel"]))
        tb_mass.set_val(str(int(p["mass"])))
        apply_values(p["speed"], p["decel"], p["mass"],
                     colors={"body": p["body_color"], "edge": p["edge_color"],
                             "cab":  p["cab_color"],  "win":  p["win_color"]})

    def on_bullet(_e):
        p = PRESETS["BULLET"]
        tb_speed.set_val(str(int(p["speed"])))
        tb_decel.set_val(str(p["decel"]))
        tb_mass.set_val(str(int(p["mass"])))
        apply_values(p["speed"], p["decel"], p["mass"],
                     colors={"body": p["body_color"], "edge": p["edge_color"],
                             "cab":  p["cab_color"],  "win":  p["win_color"]})

    def on_run(_e):
        try:
            s = float(tb_speed.text); a = float(tb_decel.text); m = float(tb_mass.text)
            if s <= 0 or a <= 0 or m <= 0:
                print("  All values must be positive."); return
        except ValueError:
            print("  Invalid value – numbers only."); return
        apply_values(s, a, m)

    def on_eddy(_e):
        sld_ref[0] = open_eddy_window(
            state["speed_kmh"], state["mass_t"], state["decel"])

    btn_indian.on_clicked(on_indian)
    btn_bullet.on_clicked(on_bullet)
    btn_run.on_clicked(on_run)
    btn_eddy.on_clicked(on_eddy)

    plt.show()
    return state["anim"]


# ─────────────────────────────────────────────────────────────────────────────
# Parametric Batch Simulation (70-100 Conditions Analysis)
# ─────────────────────────────────────────────────────────────────────────────
def run_batch_simulation(n_conditions=100, seed=42, output_dir="."):
    """
    Run parametric batch simulation across 70-100 different operational conditions.
    Computes summary statistics: mean, standard deviation, min, max, median.
    Exports CSV and multi-panel visualization plot.
    """
    if seed is not None:
        np.random.seed(seed)

    print(f"\n{'='*60}")
    print(f"RUNNING PARAMETRIC BATCH SIMULATION: {n_conditions} OPERATING CONDITIONS")
    print(f"{'='*60}")

    speeds = np.random.uniform(80.0, 220.0, n_conditions)
    masses = np.random.uniform(200.0, 800.0, n_conditions)
    conv_decels = np.random.uniform(0.50, 1.10, n_conditions)
    eddy_addons = np.random.uniform(0.15, 0.55, n_conditions)

    results = []
    for i in range(n_conditions):
        s = speeds[i]
        m = masses[i]
        a_conv = conv_decels[i]
        a_eddy = eddy_addons[i]
        a_total = a_conv + a_eddy

        v0 = s / 3.6
        m_kg = m * 1000.0

        t_conv = v0 / a_conv
        d_conv = (v0 ** 2) / (2.0 * a_conv)
        W_total = 0.5 * m_kg * (v0 ** 2)

        t_eddy = v0 / a_total
        d_eddy = (v0 ** 2) / (2.0 * a_total)

        d_saved = d_conv - d_eddy
        d_saved_pct = (d_saved / d_conv) * 100.0
        t_saved = t_conv - t_eddy
        t_saved_pct = (t_saved / t_conv) * 100.0

        W_eddy_frac = a_eddy / a_total
        W_eddy_raw = W_total * W_eddy_frac
        W_regen_J = W_eddy_raw * REGEN_EFFICIENCY
        regen_kwh = W_regen_J / 3.6e6
        regen_pct = (W_regen_J / W_total) * 100.0
        co2_saved = regen_kwh * CO2_PER_KWH
        homes_days = regen_kwh / HOME_KWH_PER_DAY

        results.append({
            'Run_ID': i + 1,
            'Speed_kmh': round(s, 2),
            'Mass_tonnes': round(m, 2),
            'Conv_Decel_mps2': round(a_conv, 3),
            'Eddy_Addon_mps2': round(a_eddy, 3),
            'Total_Decel_mps2': round(a_total, 3),
            'Stop_Time_Conv_s': round(t_conv, 2),
            'Stop_Dist_Conv_m': round(d_conv, 2),
            'Stop_Time_Augmented_s': round(t_eddy, 2),
            'Stop_Dist_Augmented_m': round(d_eddy, 2),
            'Dist_Saved_m': round(d_saved, 2),
            'Dist_Saved_Pct': round(d_saved_pct, 2),
            'Time_Saved_s': round(t_saved, 2),
            'Total_Energy_MJ': round(W_total / 1e6, 2),
            'Regen_Energy_kWh': round(regen_kwh, 2),
            'Regen_Energy_MJ': round(W_regen_J / 1e6, 2),
            'Energy_Recovery_Pct': round(regen_pct, 2),
            'CO2_Saved_kg': round(co2_saved, 2),
            'Homes_Days': round(homes_days, 2)
        })

    df = pd.DataFrame(results)
    csv_file = os.path.join(output_dir, f"simulation_{n_conditions}_conditions.csv")
    df.to_csv(csv_file, index=False)

    summary_cols = ['Speed_kmh', 'Mass_tonnes', 'Conv_Decel_mps2', 'Eddy_Addon_mps2', 'Total_Decel_mps2', 
                    'Stop_Dist_Conv_m', 'Stop_Dist_Augmented_m', 'Dist_Saved_m', 'Dist_Saved_Pct',
                    'Stop_Time_Conv_s', 'Stop_Time_Augmented_s', 'Time_Saved_s',
                    'Total_Energy_MJ', 'Regen_Energy_kWh', 'Energy_Recovery_Pct', 'CO2_Saved_kg']
    stats = df[summary_cols].agg(['mean', 'std', 'min', 'max', 'median']).round(2)
    stats_file = os.path.join(output_dir, "simulation_statistics_summary.csv")
    stats.to_csv(stats_file)

    # Generate 4-panel analysis plot
    fig, axes = plt.subplots(2, 2, figsize=(14, 10), facecolor='#0d1117')
    fig.suptitle(f'Multi-Condition Parametric Study (N = {n_conditions} Scenarios)', color='white', fontsize=15, fontweight='bold')

    for ax in axes.flat:
        ax.set_facecolor('#161b22')
        ax.tick_params(colors='#8b949e', labelsize=9)
        for sp in ax.spines.values():
            sp.set_edgecolor('#30363d')
        ax.grid(True, color='#21262d', linestyle='--', alpha=0.6)

    mean_conv = df['Stop_Dist_Conv_m'].mean()
    mean_aug = df['Stop_Dist_Augmented_m'].mean()
    axes[0, 0].hist(df['Stop_Dist_Conv_m'], bins=15, alpha=0.6, color='#58a6ff', label=f'Conventional (Mean: {mean_conv:.0f}m)')
    axes[0, 0].hist(df['Stop_Dist_Augmented_m'], bins=15, alpha=0.7, color='#3fb950', label=f'Augmented / Regen (Mean: {mean_aug:.0f}m)')
    axes[0, 0].set_title('Stopping Distance Distribution', color='white', fontsize=11)
    axes[0, 0].set_xlabel('Stopping Distance (m)', color='#8b949e', fontsize=9)
    axes[0, 0].set_ylabel('Frequency', color='#8b949e', fontsize=9)
    axes[0, 0].legend(facecolor='#161b22', edgecolor='#30363d', labelcolor='white', fontsize=8.5)

    axes[0, 1].scatter(df['Speed_kmh'], df['Stop_Dist_Conv_m'], color='#58a6ff', alpha=0.7, label='Conventional')
    axes[0, 1].scatter(df['Speed_kmh'], df['Stop_Dist_Augmented_m'], color='#3fb950', alpha=0.7, label='Augmented (Eddy + Regen)')
    axes[0, 1].set_title('Speed vs Stopping Distance', color='white', fontsize=11)
    axes[0, 1].set_xlabel('Initial Speed (km/h)', color='#8b949e', fontsize=9)
    axes[0, 1].set_ylabel('Stopping Distance (m)', color='#8b949e', fontsize=9)
    axes[0, 1].legend(facecolor='#161b22', edgecolor='#30363d', labelcolor='white', fontsize=8.5)

    mean_pct = df['Dist_Saved_Pct'].mean()
    axes[1, 0].hist(df['Dist_Saved_Pct'], bins=15, color='#f0883e', edgecolor='#161b22', alpha=0.85)
    axes[1, 0].axvline(mean_pct, color='white', linestyle='--', linewidth=1.5, label=f'Mean: {mean_pct:.1f}%')
    axes[1, 0].set_title('Stopping Distance Reduction Percentage', color='white', fontsize=11)
    axes[1, 0].set_xlabel('Distance Reduction (%)', color='#8b949e', fontsize=9)
    axes[1, 0].set_ylabel('Frequency', color='#8b949e', fontsize=9)
    axes[1, 0].legend(facecolor='#161b22', edgecolor='#30363d', labelcolor='white', fontsize=8.5)

    sc = axes[1, 1].scatter(df['Speed_kmh'], df['Regen_Energy_kWh'], c=df['Mass_tonnes'], cmap='viridis', alpha=0.85, s=45)
    cbar = plt.colorbar(sc, ax=axes[1, 1])
    cbar.set_label('Train Mass (tonnes)', color='#8b949e', fontsize=9)
    cbar.ax.yaxis.set_tick_params(color='#8b949e')
    plt.setp(plt.getp(cbar.ax.axes, 'yticklabels'), color='#8b949e')
    axes[1, 1].set_title('Regenerated Energy vs Speed & Mass', color='white', fontsize=11)
    axes[1, 1].set_xlabel('Initial Speed (km/h)', color='#8b949e', fontsize=9)
    axes[1, 1].set_ylabel('Regenerated Energy (kWh)', color='#8b949e', fontsize=9)

    plt.tight_layout()
    plot_file = os.path.join(output_dir, f"batch_{n_conditions}_conditions_analysis.png")
    plt.savefig(plot_file, dpi=200, facecolor=fig.get_facecolor(), edgecolor='none')
    plt.close(fig)

    print(f"\n{'='*60}")
    print(f"BATCH SIMULATION COMPLETE ({n_conditions} CONDITIONS)")
    print(f"{'='*60}")
    print(f"Detailed run log saved to : {csv_file}")
    print(f"Statistical summary saved: {stats_file}")
    print(f"Visualization plot saved : {plot_file}")
    print(f"\nSTATISTICAL SUMMARY (Mean, Std Dev, Min, Max, Median):")
    print(stats.T.to_string())
    print(f"{'='*60}\n")
    return df, stats


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Train Braking Simulation & Batch Analysis")
    parser.add_argument("--batch", "-b", nargs="?", const=100, type=int, default=None,
                        help="Run batch simulation across N conditions (default: 100)")
    parser.add_argument("--outdir", "-o", type=str, default=".",
                        help="Output directory for batch results")
    args, unknown = parser.parse_known_args()

    if args.batch is not None:
        run_batch_simulation(n_conditions=args.batch, output_dir=args.outdir)
        return

    speed_kmh, decel, mass_t = get_inputs()
    run_simulation(speed_kmh, decel, mass_t)


if __name__ == "__main__":
    main()
