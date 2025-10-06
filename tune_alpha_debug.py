# tune_alpha_debug.py
# Debug script to verify gyro integration and bias logic.
# Run: python tune_alpha_debug.py

import math, types
from imu import IMUGui

def debug_for_alpha(alpha_cf, rot_deg_per_s=5.0, bias_window_secs=2.0, dt=0.1):
    print(f"\n--- DEBUG alpha_cf={alpha_cf} ---")
    D = type("D", (), {})()
    # minimal state the methods expect
    D._gyro_t0 = None
    D._gyro_bias_accum = [0.0, 0.0, 0.0]
    D._gyro_bias_count = 0
    D._gyro_bias = [0.0, 0.0, 0.0]
    D._gyro_bias_finalized = False
    D._gyro_prev_ts = None
    D._compl_prev_ts = None
    D.alpha_cf = alpha_cf
    D.alphaGInt = D.betaGInt = D.alphaCInt = D.betaCInt = 0.0

    # bind methods
    D.processAccel = types.MethodType(IMUGui.processAccel, D)
    D.processGyro  = types.MethodType(IMUGui.processGyro, D)
    D.processCompl = types.MethodType(IMUGui.processCompl, D)

    # 1) Stationary period: accumulate bias
    t = 0.0
    n_stationary = int(round(bias_window_secs / dt))
    print("Stationary phase (bias accumulation):")
    for i in range(n_stationary + 1):  # +1 to ensure we cross the window
        accel = [0.0, 0.0, 1.0]
        gyro = [0.0, 0.0, 0.0]
        # call processGyro to let bias accumulate and possibly integrate (but gyro=0)
        D.processGyro(t, gyro)
        print(f" t={t:.2f}s  bias_count={D._gyro_bias_count}  bias_accum={D._gyro_bias_accum}")
        t += dt

    # If bias finalized, print bias
    if D._gyro_bias_finalized:
        print("Bias finalized:", D._gyro_bias)
    else:
        print("Bias not finalized yet; _gyro_t0=", D._gyro_t0, " last t=", t)

    # 2) Rotation phase: call processGyro (direct) to check integration
    gx = rot_deg_per_s * math.pi / 180.0
    print("\nRotation phase (calling processGyro directly):")
    n_rot = int(round(1.0 / dt))
    for i in range(n_rot):
        # call processGyro with a rotation about X
        before = getattr(D, "alphaGInt", 0.0)
        D.processGyro(t, [gx, 0.0, 0.0])
        after = getattr(D, "alphaGInt", 0.0)
        dt_local = 0.0 if D._gyro_prev_ts is None else 0.0  # we will compute from differences differently
        # compute printed dt as the increment we provided (should be dt)
        # (we can't directly access previous timestamp before call, so approximate)
        print(f" t={t:.2f}s  gx(rad/s)={gx:.6f}  alphaGInt_before(deg)={before*(180.0/math.pi):.4f}  after(deg)={after*(180.0/math.pi):.4f}")
        t += dt

    print("\nFinal gyro-integrated alpha (deg):", getattr(D, "alphaGInt", 0.0)*(180.0/math.pi))
    return getattr(D, "alphaGInt", 0.0)*(180.0/math.pi)

if __name__ == "__main__":
    candidates = [0.90, 0.95, 0.98]
    for k in candidates:
        debug_for_alpha(k)
    print("\nDebug run complete.")
