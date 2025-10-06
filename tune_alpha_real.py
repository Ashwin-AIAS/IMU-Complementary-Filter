# tune_alpha_real.py
# Sweep alpha_cf on the real dataset and compute RMSE vs accelerometer angle.
# Run: python tune_alpha_real.py imudata.txt

import sys, math, types
from imu import IMUGui, DataReader

def evaluate_on_file(filename, alphas, ignore_initial_secs=2.0):
    reader = DataReader(filename)
    results = []
    # Read whole file into memory once (timestamp + accel + gyro)
    rows = []
    while True:
        v = reader.readDataLine(7)
        if v is None:
            break
        rows.append(v)
    if len(rows) == 0:
        raise RuntimeError("No data read from file.")

    for alfa in alphas:
        # Create object with minimal state; bind IMUGui methods
        D = type("D", (), {})()
        D._gyro_t0 = None
        D._gyro_bias_accum = [0.0, 0.0, 0.0]
        D._gyro_bias_count = 0
        D._gyro_bias = [0.0, 0.0, 0.0]
        D._gyro_bias_finalized = False
        D._gyro_prev_ts = None
        D._compl_prev_ts = None
        D.alpha_cf = alfa
        D.alphaGInt = D.betaGInt = D.alphaCInt = D.betaCInt = 0.0

        D.processAccel = types.MethodType(IMUGui.processAccel, D)
        D.processGyro  = types.MethodType(IMUGui.processGyro,  D)
        D.processCompl = types.MethodType(IMUGui.processCompl, D)

        sq_err_alpha = 0.0
        sq_err_beta = 0.0
        n = 0

        for row in rows:
            t = row[0]
            accel = row[1:4]
            # convert gyro deg/s -> rad/s
            gyro = [v * math.pi/180.0 for v in row[4:7]]

            # compute accel angles (reference)
            a_alpha, a_beta, _ = D.processAccel(t, accel)
            # run complementary
            c_alpha, c_beta, _ = D.processCompl(t, accel, gyro)

            # after initial ignore window (to allow bias finalize), collect error
            if D._gyro_t0 is not None and t >= (D._gyro_t0 + ignore_initial_secs):
                da = c_alpha - a_alpha
                db = c_beta - a_beta
                sq_err_alpha += da*da
                sq_err_beta += db*db
                n += 1

        if n > 0:
            rmse_alpha = math.sqrt(sq_err_alpha / n)
            rmse_beta = math.sqrt(sq_err_beta / n)
        else:
            rmse_alpha = float("nan")
            rmse_beta = float("nan")

        results.append((alfa, rmse_alpha, rmse_beta))
        print(f"alpha={alfa:.3f}  RMSE_alpha={rmse_alpha:.3f}°  RMSE_beta={rmse_beta:.3f}° (n={n})")
    return results

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python tune_alpha_real.py imudata.txt")
        sys.exit(1)
    fname = sys.argv[1]
    alphas = [0.90, 0.95, 0.98, 0.99, 0.995]
    evaluate_on_file(fname, alphas)
