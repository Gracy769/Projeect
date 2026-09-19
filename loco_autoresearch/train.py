from prepare import run_simulation

def braking_controller(v_ms, t_s):
    """
    Returns the target slip for the induction motor inverter.
    v_ms: Current train speed in m/s
    t_s: Time elapsed in seconds
    
    Valid slip for regeneration is typically between -0.01 and -0.15.
    (More negative slip = more braking torque, up to breakdown slip).
    """
    # Baseline controller: constant slip
    target_slip = -0.15
    return target_slip

if __name__ == "__main__":
    run_simulation(braking_controller)
