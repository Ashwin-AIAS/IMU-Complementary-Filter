# tune_alpha.py
# Sweep a set of complementary filter alpha_cf values using IMUGui.
# Uses imu_new.IMUGui (the file you just tested). If imu_new doesn't exist,
# it will try to import imu.IMUGui.
#
# Usage:
#   python tune_alpha.py
#   python tune_alpha.py --real imudata.txt

import math
import sys
import importlib

# prefer imu_new (your updated file), fall back to imu if not present
try:
    imu_mod = importlib.import_module("imu_new")
except Exception:
    imu_mod = importlib.import_module("imu")

IMUGui = imu_mod.IMUGui

def sim_self_test(alpha_cf, rot_deg_per_s=5.0, bias_window_secs=2.0, dt=0.1):
    """
    Simulate the same self-test you used: bias_window_secs stationary,
    then 1s rotation at rot_deg_per_s.
    Returns measured complementary alpha (deg) and gyro-integrated alpha (deg).
    """
    gui = IMUGui(None)
    gui.alpha_cf = alpha_cf

    t = 0.0
    # stationary
    n_stat = int(round(bias_window_secs / dt))
    for _ in range(n_stat + 1):
        accel = [0.0, 0.0, 1.0]
        gyro = [0.0, 0.0, 0.0]
        # note: processCompl expects gyro in rad/s (IMUGui expects rad/s)
        gui.processCompl(t, accel, gyro)
        t += dt

    # rotation
    gx = rot_deg_per_s * math.pi / 180.0
    res_alpha_deg = 0.0
    for _ in range(int(round(1.0/dt))):
        accel = [0.0, 0.0, 1.0]
        gyro = [gx, 0.0, 0.0]
        res_alpha_deg, res_beta_deg, _ = gui.processCompl(t, accel, gyro)
        t += dt

    # gyro-integrated (from complementary integrator alphaCInt) in degrees
    gyro_alpha_deg = gui.alphaCInt * 180.0 / math.pi
    return res_alpha_deg, gyro_alpha_deg

def evaluate_real_file(filename, alpha_cf, ignore_initial_secs=2.0):
    """
    Run through real data file and compute RMSE of compl result vs accel angles
    (accel used as long-term reference). This is only an indicator.
    """
    import math
    from imu_mod import DataReader  # attempt to use whichever module has DataReader
    # fallback if attribute not present
    DR = None
    if hasattr(imu_mod, "DataReader"):
        DR = imu_mod.DataReader
    else:
        # try to import a DataReader in the top-level imu.py
        try:
            tmp = importlib.import_module("imu")
            DR = tmp.DataReader
        except Exception:
            raise RuntimeError("No DataReader found in imu_new or imu modules.")

    reader = DR(filename)
    gui = IMUGui(reader)
    gui.alpha_cf = alpha_cf

    # read all rows first (simple)
    rows = []
    while True:
        row = reader.readDataLine(7)
        if row is None:
            break
        rows.append(row)
    if len(rows) == 0:
        return (float("nan"), float("nan"), 0)

    sq_err_a = 0.0
    sq_err_b = 0.0
    n = 0
    for row in rows:
        t = row[0]
        accel = row[1:4]
        gyro_deg = row[4:7]
        # convert to rad/s
        gyro = [v * math.pi / 180.0 for v in gyro_deg]

        a_alpha_deg, a_beta_deg, _ = gui.processAccel(t, accel)
        c_alpha_deg, c_beta_deg, _ = gui.processCompl(t, accel, gyro)

        # collect errors after initial ignore window
        if gui._gyro_t0 is not None and t >= gui._gyro_t0 + ignore_initial_secs:
            da = c_alpha_deg - a_alpha_deg
            db = c_beta_deg - a_beta_deg
            sq_err_a += da*da
            sq_err_b += db*db
            n += 1

    if n == 0:
        return (float("nan"), float("nan"), 0)
    rmse_a = math.sqrt(sq_err_a / n)
    rmse_b = math.sqrt(sq_err_b / n)
    return (rmse_a, rmse_b, n)

def main():
    candidates = [0.90, 0.95, 0.98, 0.99, 0.995]
    print("Simulated self-test sweep (2s bias + 1s 5deg/s rotation):")
    print("alpha_cf, compl_alpha_deg, gyro_alpha_deg, error_vs_expected(5deg)")
    for a in candidates:
        compl_alpha, gyro_alpha = sim_self_test(a)
        err = compl_alpha - 5.0
        print(f"{a:.3f}, {compl_alpha:.3f}, {gyro_alpha:.3f}, {err:+.3f}")

    # optionally evaluate on real file if requested
    if len(sys.argv) >= 3 and sys.argv[1] == "--real":
        fname = sys.argv[2]
        print("\nEvaluating on real file:", fname)
        print("alpha_cf, RMSE_alpha(deg), RMSE_beta(deg), samples")
        for a in candidates:
            rmse_a, rmse_b, n = evaluate_real_file(fname, a)
            print(f"{a:.3f}, {rmse_a:.3f}, {rmse_b:.3f}, {n}")

if __name__ == "__main__":
    main()
