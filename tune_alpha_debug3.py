# tune_alpha_debug3.py
# Force _gyro_t0 so bias accumulation runs, then test integration.
# Run: python tune_alpha_debug3.py

import math
from imu import IMUGui

def debug_for_alpha_forced(alpha_cf, rot_deg_per_s=5.0, bias_window_secs=2.0, dt=0.1):
    print(f"\n--- DEBUG (forced _gyro_t0) alpha_cf={alpha_cf} ---")
    gui = IMUGui(None)

    # Force initial state
    gui.alpha_cf = alpha_cf

    # Force a starting _gyro_t0 so the bias window logic will accumulate
    gui._gyro_t0 = 0.0
    gui._gyro_bias_accum = [0.0, 0.0, 0.0]
    gui._gyro_bias_count = 0
    gui._gyro_bias = [0.0, 0.0, 0.0]
    gui._gyro_bias_finalized = False
    gui._gyro_prev_ts = None
    gui._compl_prev_ts = None
    gui.alphaGInt = 0.0
    gui.betaGInt = 0.0
    gui.alphaCInt = 0.0
    gui.betaCInt = 0.0

    # Stationary phase (should accumulate bias_count)
    t = 0.0
    n_stationary = int(round(bias_window_secs / dt))
    print("Stationary phase (bias accumulation):")
    for i in range(n_stationary + 1):
        gyro = [0.0, 0.0, 0.0]
        gui.processGyro(t, gyro)
        print(f" t={t:.2f}s  bias_count={gui._gyro_bias_count}  bias_accum={gui._gyro_bias_accum}")
        t += dt

    # finalize info
    print("After stationary: _gyro_bias_finalized =", gui._gyro_bias_finalized)
    if gui._gyro_bias_finalized:
        print("Computed bias:", gui._gyro_bias)

    # Rotation phase: integrate gyro directly
    print("\nRotation phase (calling processGyro directly):")
    gx = rot_deg_per_s * math.pi / 180.0
    n_rot = int(round(1.0 / dt))
    for i in range(n_rot):
        before_deg = gui.alphaGInt * (180.0 / math.pi)
        gui.processGyro(t, [gx, 0.0, 0.0])
        after_deg = gui.alphaGInt * (180.0 / math.pi)
        print(f" t={t:.2f}s  gx(rad/s)={gx:.6f}  before(deg)={before_deg:.4f}  after(deg)={after_deg:.4f}")
        t += dt

    print("\nFinal gyro-integrated alpha (deg):", gui.alphaGInt * (180.0 / math.pi))
    return gui.alphaGInt * (180.0 / math.pi)

if __name__ == "__main__":
    for k in [0.90, 0.98]:
        debug_for_alpha_forced(k)
    print("\nForced debug run complete.")
