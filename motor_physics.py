"""
Phase Induction Motor — Regenerative Braking Physics Engine
=============================================================
Models a 3-phase squirrel-cage induction motor acting as the traction
motor of a locomotive, and shows how it produces braking torque and
regenerates energy when its slip goes negative (motor -> generator).

Core physics (Kloss's / Simplified Torque-Slip Equation):

    Ns   = 120*f / P                      synchronous speed   (rpm)
    s    = (Ns - Nr) / Ns                 slip (dimensionless)
    T(s) = 2*Tmax / (s/sm + sm/s)         torque-slip curve (Kloss)

Where:
    f    = supply frequency (Hz)               -- set by traction inverter
    P    = number of poles
    Nr   = rotor mechanical speed (rpm)
    sm   = slip at maximum (breakdown) torque
    Tmax = breakdown (pull-out) torque (N.m)

Sign convention used here:
    s > 0   -> MOTORING     (rotor slower than field, torque drives train)
    s = 0   -> SYNCHRONOUS  (no torque, "floats")
    s < 0   -> REGENERATION (rotor faster than field -> motor becomes an
                              induction GENERATOR, torque reverses and
                              brakes the train, energy flows back to the
                              supply/catenary)
    s > 1   -> PLUGGING     (reversed phase sequence forces braking too,
                              but ALL energy is dissipated as heat in the
                              rotor -- no regeneration. Shown for contrast.)

During braking, the traction inverter deliberately lowers the supply
frequency f (hence Ns) below the train's actual mechanical speed. This
forces s negative on purpose -- it is how real locomotives (Vande
Bharat, EMUs, metros) brake regeneratively using the SAME motors that
drive them.
"""

import numpy as np

# ---------------------------------------------------------------
# Physical / motor constants (typical traction induction motor)
# ---------------------------------------------------------------
POLES = 4                  # pole count (P) -- typical traction motor
SLIP_MAX_TORQUE = 0.15     # sm: slip at breakdown torque (typical 0.1-0.2)
WHEEL_RADIUS = 0.46        # m -- typical locomotive wheel radius
GEAR_RATIO = 5.5           # motor shaft turns per wheel turn (typical traction gearbox)
MOTORS_PER_TRAIN = 8       # number of traction motors sharing the load


def synchronous_speed_rpm(freq_hz, poles=POLES):
    """Ns = 120f/P  (rpm)"""
    return 120.0 * freq_hz / poles


def slip(ns_rpm, nr_rpm):
    """s = (Ns - Nr) / Ns"""
    if ns_rpm == 0:
        return 0.0
    return (ns_rpm - nr_rpm) / ns_rpm


def kloss_torque(s, t_max, s_max, eps=1e-6):
    """
    Kloss's equation (simplified torque-slip curve):
        T(s) = 2*Tmax / (s/sm + sm/s)

    Valid for all s != 0 (s=0 handled as a limit -> T=0).
    Works for s<0 (regen), 0<s<1 (motoring), s>1 (plugging).
    """
    s = np.asarray(s, dtype=float)
    s_safe = np.where(np.abs(s) < eps, eps, s)
    return 2.0 * t_max / (s_safe / s_max + s_max / s_safe)


def wheel_speed_rpm(train_speed_ms, wheel_radius=WHEEL_RADIUS, gear_ratio=GEAR_RATIO):
    """Convert linear train speed -> equivalent MOTOR shaft rpm (through gearbox)."""
    wheel_omega = train_speed_ms / wheel_radius          # rad/s at wheel
    wheel_rpm = wheel_omega * 60.0 / (2 * np.pi)
    return wheel_rpm * gear_ratio                        # motor shaft rpm


def motor_braking_force(train_speed_ms, inverter_freq_hz, t_max, n_motors=MOTORS_PER_TRAIN,
                         wheel_radius=WHEEL_RADIUS, gear_ratio=GEAR_RATIO, poles=POLES,
                         s_max=SLIP_MAX_TORQUE):
    """
    Given the train's current speed and the inverter's commanded frequency
    (which the traction control system sets BELOW the natural motor speed
    to force negative slip and brake), return:

        s          -- slip
        T_motor    -- torque per motor (N.m), negative = braking
        F_brake    -- total retarding FORCE at the wheel (N), always >= 0
        regen      -- bool, True if this operating point is regenerative (s<0)
    """
    nr = wheel_speed_rpm(train_speed_ms, wheel_radius, gear_ratio)
    ns = synchronous_speed_rpm(inverter_freq_hz, poles)
    s = slip(ns, nr)
    t_motor = kloss_torque(s, t_max, s_max)   # N.m, negative in regen region

    # Torque -> force at wheel: F = T * gear_ratio / wheel_radius, per motor, summed
    f_per_motor = t_motor * gear_ratio / wheel_radius
    f_total = f_per_motor * n_motors

    # Braking force opposes motion -> magnitude used as deceleration contribution
    f_brake = np.maximum(0.0, -f_total)  # only keep the braking (negative torque) part
    regen = s < 0
    return s, t_motor, f_brake, regen


def optimal_inverter_freq_for_braking(train_speed_ms, target_slip, poles=POLES,
                                       wheel_radius=WHEEL_RADIUS, gear_ratio=GEAR_RATIO):
    """
    Inverse problem: what inverter frequency should the traction control
    system command RIGHT NOW to hold a desired (negative) slip for smooth,
    controlled regenerative braking as the train decelerates?

        s = (Ns - Nr)/Ns  =>  Ns = Nr / (1 - s)
        Ns = 120f/P       =>  f  = Ns * P / 120
    """
    nr = wheel_speed_rpm(train_speed_ms, wheel_radius, gear_ratio)
    ns = nr / (1.0 - target_slip)
    return ns * poles / 120.0


def regen_power_watts(t_motor, nr_rpm, n_motors=MOTORS_PER_TRAIN):
    """
    Mechanical power extracted from the shaft during regen = T * omega.
    In the regen region T is negative (opposing rotor) and omega > 0,
    so P is negative from the ROTOR's perspective (power flows OUT of
    the mechanical shaft, INTO the electrical side) -- we return the
    magnitude of that recovered electrical-side power per train.
    """
    omega_r = nr_rpm * 2 * np.pi / 60.0
    p_per_motor = np.abs(t_motor * omega_r)
    return p_per_motor * n_motors  # Watts, magnitude of regenerated power
