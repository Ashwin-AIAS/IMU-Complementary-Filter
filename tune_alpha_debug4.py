# tune_alpha_debug4.py
# Force the necessary internal state so dt != 0 and integration runs.
# Run: python tune_alpha_debug4.py

import math
from imu import IMUGui

def forced_integr_test(rot_deg_per_s=5.0, dt=0.1, bias_window_secs=2.0):
    gui = IMUGui(None)

    # Force state: treat bias as already estimated (zero bias) and set prev timestamp
    gui._gyro_t0 = 0.0
    gui._gyro_bias = [0.0, 0.0, 0.0]
    gui._gyro_bias_finalized = True

    # set previous timestamp to the last stationary time so the first dt is dt
    last_stationary_t = bias_window_secs
    gui._gyro_prev_ts = last_stationary_t

    # reset integrators
    gui.alphaGInt = 0.0
    gui.betaGInt = 0.0

    print("Starting forced integration test")
    print(f"Initial _gyro_prev_ts = {gui._gyro_prev_ts}")

    # apply rotation for 1 second in steps of dt
    gx = rot_deg_per_s * math.pi / 180.0
    t = last_stationary_t + dt
    n = int(round(1.0 / dt))
    for i in range(n):
        before_deg = gui.alphaGInt * (180.0 / math.pi)
        gui.processGyro(t, [gx, 0.0, 0.0])
        after_deg = gui.alphaGInt * (180.0 / math.pi)
        print(f" t={t:.2f}s  dt~{dt:.2f}  gx(rad/s)={gx:.6f}  before(deg)={before_deg:.4f}  after(deg)={after_deg:.4f}")
        t += dt

    print("\nFinal integrated alpha (deg):", gui.alphaGInt * (180.0 / math.pi))

if __name__ == "__main__":
    forced_integr_test()
