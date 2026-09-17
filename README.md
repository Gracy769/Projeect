# Phase Induction Motor — Regenerative Braking Module

This extends your **Eddy Current Deceleration System for Locomotives**
project with a third braking mechanism: the locomotive's own 3-phase
induction traction motors, run in their **negative-slip (regenerative)**
region.

## Why this fits the existing project

Your PPT already states that electric locomotives use their traction
motors for **regenerative braking** (30–35% energy recovery) as the
real-world complement to eddy-current braking. This module makes that
statement into an actual working simulation, using the same style as
your eddy-current window: a slider-driven interactive model with
stopping time, distance, regen energy, CO2 saved, and homes powered.

## Physics — Kloss's Torque-Slip Equation

    Ns   = 120*f / P                  synchronous speed (rpm)
    s    = (Ns - Nr) / Ns             slip
    T(s) = 2*Tmax / (s/sm + sm/s)     torque-slip curve

- **s > 0** → motoring (rotor slower than field, torque drives the train)
- **s = 0** → no torque
- **s < 0** → **regeneration**: the traction inverter deliberately lowers
  its output frequency below the train's actual mechanical speed. This
  forces negative slip, which flips the torque sign — the "motor"
  becomes an induction **generator**, producing braking torque AND
  feeding electrical power back into the catenary/grid.
- **s > 1** → *plugging* (reversed phase sequence) also brakes, but every
  joule becomes heat — no regeneration. Mentioned in the code/report for
  contrast with true regen braking.

This torque then converts to a wheel-level braking force via the gear
ratio and wheel radius, and is added to the conventional friction
deceleration — exactly the same additive pattern your eddy-current
slider already uses (`total_decel = conv_decel + addon`).

## Files

| File | What it does |
|---|---|
| `motor_physics.py` | Core physics: Kloss torque-slip curve, slip calculation, force/power conversion. Imported by the sim. |
| `induction_motor_braking_sim.py` | Interactive Python GUI (Matplotlib) — run `python induction_motor_braking_sim.py`. Slider controls target regen slip; INDIAN/BULLET presets match your existing ones. |
| `induction_motor_braking.m` | MATLAB script, same `input()`/`ode45` style as your existing eddy-current `.m` file. Run it in MATLAB and answer the prompts. |

## Requirements

- Python: `numpy`, `matplotlib` (same as your existing Python sim)
- MATLAB: R2021+ (same as your existing MATLAB script), no extra toolboxes needed

## Realistic parameter choices

Per-motor breakdown torque (`Tmax`) is set to **1800–2200 N·m**, typical
for a traction induction motor of this class. This was deliberately
chosen so the motor's regen contribution **supplements** conventional
friction braking (roughly 20–30% extra deceleration, 20–30% energy
recovery) rather than dominating it — matching the real-world 30–35%
regen figure already cited in your presentation, rather than an
unrealistically large number.

## Suggested places to use this in your report/PPT

1. **Working principle slide** — add "Phase Induction Motor Regenerative
   Braking" alongside Faraday's/Lenz's Law slide, since it's a
   *different* physical mechanism (electromagnetic torque reversal via
   slip, not induced eddy currents in a passive conductor).
2. **Objectives** — add: "Model induction-motor regenerative braking via
   the torque-slip (Kloss) equation and quantify its energy recovery
   alongside eddy-current braking."
3. **Applications — Electric Locomotives slide** — this IS the
   mechanism behind the "~35% Energy recovered" stat already on that
   slide; this module shows the physics underneath that number.
4. **Comparison table** — add a third column/row: Induction Motor Regen
   vs Eddy Current vs Conventional Friction.
