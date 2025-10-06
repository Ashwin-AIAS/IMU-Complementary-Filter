# tune_alpha_debug2.py
# Better debug script that instantiates IMUGui to ensure proper init
# Run: python tune_alpha_debug2.py

import math
from imu import IMUGui

def debug_for_alpha(alpha_cf, rot_deg_per_s=5.0, bias_window_secs=2.0, dt=0.1):
    print(f"\n--- DEBUG alpha_cf={alpha_cf} ---")
    # instantiate real IMUGui (pass None as DataReader since we won't use it)
    gui = IMUGui(None)
    # set alpha_cf and reset relevant internal state
    gui.alpha_cf = alpha_cf
    gui._gyro_t0 = None
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

    # Stationary (bias) period
    t = 0.0
    n_stationary = int(round(bias_window_secs / dt))
    print("Stationary phase (bias accumulation):")
    for i in range(n_stationary + 1):  # +1 to step past the window
        gyro = [0.0, 0.0, 0.0]
        gui.processGyro(t, gyro)
        print(f" t={t:.2f}s  bias_count={gui._gyro_bias_count}  bias_accum={gui._gyro_bias_accum}")
        t += dt

    if gui._gyro_bias_finalized:
        print(" Bias finalized:", gui._gyro_bias)
    else:
        print(" Bias not finalized yet; _gyro_t0=", gui._gyro_t0, " last t=", t)

    # Rotation phase: call processGyro directly to test integration
    gx = rot_deg_per_s * math.pi / 180.0
    print("\nRotation phase (calling processGyro directly):")
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
    candidates = [0.90, 0.95, 0.98]
    for k in candidates:
        debug_for_alpha(k)
    print("\nDebug run complete.")
