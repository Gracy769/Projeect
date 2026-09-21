# Locomotive Regenerative & Eddy-Current Braking Optimization

A multi-domain research, computational simulation, and autonomous optimization framework for **3-Phase Induction Motor Regenerative Braking** and **Eddy-Current Deceleration Systems** in heavy passenger and high-speed rail.

Includes interactive simulation GUIs (MATLAB & Python), an autonomous AI optimization engine (`loco_autoresearch`), a 100-scenario Monte Carlo parametric sensitivity study, and a full publication-grade research paper.

---

## 📑 Research Paper & Presentation PDF

- 📄 **Compiled Presentation PDF**: [`loco_autoresearch/Research_Paper_Braking_Optimization.pdf`](./loco_autoresearch/Research_Paper_Braking_Optimization.pdf)
- 📝 **Full Research Paper (Markdown)**: [`loco_autoresearch/Research_Paper.md`](./loco_autoresearch/Research_Paper.md)

### Key Findings Summary ($N = 100$ Operating Conditions):
- **Stopping Distance Saved**: **$350.95 \pm 248.14\text{ m}$ ($30.06\% \pm 9.63\%$)**, up to **$1.26\text{ km}$** saved in emergency scenarios.
- **Stopping Time Saved**: **$16.33 \pm 8.93\text{ seconds}$** per deceleration event.
- **Regenerated Electrical Energy**: **$17.80 \pm 12.00\text{ kWh}$** per stop ($4.15\text{ kg CO}_2$ offset, up to $63.49\text{ kWh}$ in high-speed stops).
- **Global Slip Optimum**: Proved analytically and via autonomous AI search that continuous operation at breakdown slip ($s = -0.15$) minimizes composite stopping penalty.

---

## 📂 Repository File Structure

```text
├── README.md                                    # Project documentation & overview
├── train_simu.py                                # Integrated train simulation GUI + 100-run batch engine
├── induction_motor_braking_sim.py               # Interactive Matplotlib simulator (window 3)
├── motor_physics.py                             # Core physics engine: Kloss equation, gear ratio, power
├── induction_motor_braking.m.txt                # MATLAB state-space ODE numerical solver
└── loco_autoresearch/                           # Autonomous research & optimization module
    ├── Research_Paper_Braking_Optimization.pdf  # Final compiled 12-page research paper PDF
    ├── Research_Paper.md                        # Full paper manuscript with equations & tables
    ├── simulation_100_conditions.csv            # Raw dataset across 100 Monte Carlo test runs
    ├── simulation_statistics_summary.csv        # Summary stats (Mean, Std Dev, Min, Max, Median)
    ├── prepare.py                               # Fixed physics evaluation harness
    ├── train.py                                 # Controller script optimized by AI agent
    ├── program.md                               # Autonomous agent research instructions
    ├── results.tsv                              # Optimization iteration log
    └── figures/                                 # High-resolution simulation plots & figures
        ├── matlab_sim_dashboard.jpeg            # Figure 1: MATLAB complete dashboard
        ├── kloss_torque_slip_curve.jpeg         # Figure 2: Electrodynamic Kloss torque-slip curve
        ├── velocity_braking_profile.jpeg        # Figure 3: Deceleration velocity profile comparison
        ├── stopping_distance_vs_time.jpeg       # Figure 4: Cumulative stopping distance trajectory
        ├── energy_distribution_regen.jpeg       # Figure 5: Kinetic energy partitioning bar chart
        ├── python_train_simu_gui.jpeg           # Figure 6: Python track view animation & telemetry GUI
        └── batch_100_conditions_analysis.png    # Figure 7: 4-panel 100-condition parametric study
```

---

## ⚡ Quick Start & Usage

### 1. Run Interactive Train Braking Simulation
Launches the full interactive dark-mode dashboard with real-time train animation, track view, speed/distance graphs, and auxiliary eddy-current analysis:
```bash
python train_simu.py
```
*Presets available: `INDIAN` (160 km/h Express, 400t) and `BULLET` (200 km/h HSR, 400t).*

### 2. Run 100-Scenario Parametric Batch Analysis
Executes a Monte Carlo simulation across $N = 100$ randomized operating conditions (speed: 80–220 km/h, mass: 200–800 t, deceleration: 0.50–1.10 m/s², add-ons: 0.15–0.55 m/s²), computes summary statistics ($\mu, \sigma, \min, \max, \text{median}$), and outputs CSVs and analysis plots:
```bash
python train_simu.py --batch 100
```
Outputs generated:
- `simulation_100_conditions.csv` (Run-by-run log)
- `simulation_statistics_summary.csv` (Aggregated statistics)
- `batch_100_conditions_analysis.png` (4-panel statistical distribution)

### 3. Run Induction Motor Interactive Simulator
Interactive Matplotlib GUI with live torque-slip operating point tracking on the Kloss curve:
```bash
python induction_motor_braking_sim.py
```

### 4. Run MATLAB Simulation
Run in MATLAB (R2021+):
```matlab
run induction_motor_braking.m
```

---

## 🔬 Physics Fundamentals (Kloss Equation)

The 3-phase induction traction motor torque is modeled using Kloss's torque-slip formulation:

$$T(s) = \frac{2 T_{\max}}{\frac{s}{s_m} + \frac{s_m}{s}}$$

- **$s > 0$ (Motoring)**: Synchronous speed $N_s >$ rotor speed $N_r$. Torque drives the train forward.
- **$s = 0$ (Synchronous)**: $N_s = N_r$. Zero net torque produced.
- **$s < 0$ (Regenerative Braking)**: Inverter lowers stator frequency below rotor mechanical speed ($N_s < N_r$). Torque sign flips ($T < 0$), retarding the shaft and pumping electrical energy back into the overhead catenary.
- **$s > 1$ (Plugging)**: Reversed phase sequence; strong braking torque but dissipates all energy as rotor resistive heat.

---

## ⚠️ Engineering Limitations & Hardware Challenges

As detailed in the research paper, field deployment on existing rail lines faces physical bottlenecks:
1. **High Magnetic Flux & Thermal Hazards**: Sustained maximum torque causes high flux concentration ($B > 1.5 - 2.0\text{ T}$), creating extreme Joule heating in rails and risking component melting.
2. **Electromagnetic Interference (EMI)**: Powerful transient fields threaten trackside signaling circuits (ETCS Level 2 / Indian Railways' Kavach) and onboard low-voltage telemetry.
3. **Catenary Grid Receptivity**: In non-receptive power grids without line-side energy storage (TESS), regenerated energy risks overvolting the DC link, requiring onboard rheostatic brake resistor banks.
4. **Preliminary Nature**: Computational models establish theoretical limits; physical adoption demands full hardware-in-the-loop (HIL) dynamometer testing and track-level shielding.

---

## 🛠 Requirements

- **Python**: 3.10+ (`numpy`, `matplotlib`, `pandas`)
- **MATLAB**: R2021+ (no additional toolboxes required)
- **PDF Viewer**: Any standard viewer for [`Research_Paper_Braking_Optimization.pdf`](./loco_autoresearch/Research_Paper_Braking_Optimization.pdf)
