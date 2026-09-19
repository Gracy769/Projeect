import numpy as np
import time

# --- Motor Physics Constants ---
POLES = 4
SLIP_MAX_TORQUE = 0.15
WHEEL_RADIUS = 0.46
GEAR_RATIO = 5.5
MOTORS_PER_TRAIN = 8

def synchronous_speed_rpm(freq_hz, poles=POLES):
    return 120.0 * freq_hz / poles

def slip(ns_rpm, nr_rpm):
    if ns_rpm == 0: return 0.0
    return (ns_rpm - nr_rpm) / ns_rpm

def kloss_torque(s, t_max, s_max=SLIP_MAX_TORQUE, eps=1e-6):
    s = np.asarray(s, dtype=float)
    s_safe = np.where(np.abs(s) < eps, eps, s)
    return 2.0 * t_max / (s_safe / s_max + s_max / s_safe)

def wheel_speed_rpm(train_speed_ms, wheel_radius=WHEEL_RADIUS, gear_ratio=GEAR_RATIO):
    wheel_omega = train_speed_ms / wheel_radius
    wheel_rpm = wheel_omega * 60.0 / (2 * np.pi)
    return wheel_rpm * gear_ratio

def motor_braking_force(train_speed_ms, inverter_freq_hz, t_max, n_motors=MOTORS_PER_TRAIN,
                         wheel_radius=WHEEL_RADIUS, gear_ratio=GEAR_RATIO, poles=POLES,
                         s_max=SLIP_MAX_TORQUE):
    nr = wheel_speed_rpm(train_speed_ms, wheel_radius, gear_ratio)
    ns = synchronous_speed_rpm(inverter_freq_hz, poles)
    s = slip(ns, nr)
    t_motor = kloss_torque(s, t_max, s_max)
    f_per_motor = t_motor * gear_ratio / wheel_radius
    f_total = f_per_motor * n_motors
    f_brake = np.maximum(0.0, -f_total)
    regen = s < 0
    return s, t_motor, f_brake, regen

def optimal_inverter_freq_for_braking(train_speed_ms, target_slip, poles=POLES,
                                       wheel_radius=WHEEL_RADIUS, gear_ratio=GEAR_RATIO):
    nr = wheel_speed_rpm(train_speed_ms, wheel_radius, gear_ratio)
    ns = nr / (1.0 - target_slip)
    return ns * poles / 120.0

def regen_power_watts(t_motor, nr_rpm, n_motors=MOTORS_PER_TRAIN):
    omega_r = nr_rpm * 2 * np.pi / 60.0
    p_per_motor = np.abs(t_motor * omega_r)
    return p_per_motor * n_motors

# --- Evaluation Engine ---
REGEN_EFFICIENCY = 0.85

def run_simulation(controller_func):
    """
    Runs the braking simulation using the provided controller function.
    controller_func(v_prev, t) should return target_slip.
    """
    v0_kmh = 160.0
    mass_t = 400.0
    conv_decel = 0.8
    t_max = 1800.0
    
    v0 = v0_kmh / 3.6
    mass_kg = mass_t * 1000.0
    dt = 0.1
    max_steps = 3000
    
    t = 0.0
    v = v0
    d = 0.0
    
    t_list = [t]
    v_list = [v]
    regen_power_list = [0.0]
    
    start_time = time.time()
    
    for _ in range(max_steps):
        if v <= 0.05:
            break
            
        target_slip = controller_func(v, t)
        
        # Limit target_slip to realistic negative slip range to avoid instability
        target_slip = max(-0.99, min(0.0, target_slip))
        
        f_req = optimal_inverter_freq_for_braking(v, target_slip)
        s, T, F, regen = motor_braking_force(v, f_req, t_max)
        
        nr_rpm = wheel_speed_rpm(v)
        p_watts = regen_power_watts(T, nr_rpm) if regen else 0.0
        
        total_decel = conv_decel + F / mass_kg
        v = max(0.0, v - total_decel * dt)
        d += v * dt
        t += dt
        
        t_list.append(t)
        v_list.append(v)
        regen_power_list.append(p_watts)
        
    execution_time = time.time() - start_time
    
    W_regen_joules = np.trapezoid(regen_power_list, t_list) * REGEN_EFFICIENCY
    regen_kwh = W_regen_joules / 3.6e6
    
    # We want to minimize stop distance and maximize regen energy.
    # Score = stop_distance - regen_kwh * 20 (arbitrary weight). Lower is better.
    score = d - (regen_kwh * 20.0)
    
    print("---")
    print(f"braking_score:    {score:.6f}")
    print(f"stop_distance_m:  {d:.2f}")
    print(f"regen_kwh:        {regen_kwh:.2f}")
    print(f"stop_time_s:      {t:.2f}")
    print(f"sim_seconds:      {execution_time:.4f}")
