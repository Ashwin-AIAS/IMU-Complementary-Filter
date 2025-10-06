# imu_new.py
# IMU complementary filter implementation (Accel, Gyro, Complementary).
# - processAccel: compute roll/pitch from accel (returns degrees)
# - processGyro: integrate gyro rates (rad/s -> integrated angles in degrees), with bias estimation
# - processCompl: complementary filter combining gyro integration + accel angles
#
# Default complementary weight alpha_cf = 0.98 (tunable)
# Bias calibration window default = 2.0 seconds
#
# Usage:
#   python imu_new.py --self-test
#   python imu_new.py imudata.txt

import sys
import time
import math
import select

from DataPlot import DataPlot

class DataReader:
    def __init__(self, filename: str):
        self.file = open(filename, "r")
        self.dataFromDevice = filename.startswith("/dev/")
        self.lastTimestamp = -1.0

    def readDataLine(self, floatcount: int = -1):
        if self.dataFromDevice:
            r, w, x = select.select([self.file], [], [], 0)
            if len(r) == 0:
                return None
        line = self.file.readline()
        if not line:
            return None
        parts = line.strip().split(",")
        if floatcount != -1 and len(parts) != floatcount:
            return None
        try:
            vals = list(map(float, parts))
        except:
            return None
        # simulate real-time for recorded files
        if not self.dataFromDevice and self.lastTimestamp >= 0:
            dt = vals[0] - self.lastTimestamp
            if dt > 0:
                time.sleep(dt)
        self.lastTimestamp = vals[0]
        return vals

class IMUGui:
    def __init__(self, dataReader):
        # plotting helpers (existing DataPlot)
        self.accelPlot = DataPlot(("ax","ay","az"), 1000)
        self.gyroPlot  = DataPlot(("p","q","r"), 1000)
        self.accelResPlot = DataPlot(("alpha","beta","gamma"), 1000)
        self.gyroResPlot  = DataPlot(("alpha","beta","gamma"), 1000)
        self.complResPlot = DataPlot(("alpha","beta","gamma"), 1000)

        # gyro integration state (radians)
        self.alphaGInt = 0.0
        self.betaGInt  = 0.0

        # complementary integration state (radians)
        self.alphaCInt = 0.0
        self.betaCInt  = 0.0

        # gyro bias estimation
        self._gyro_t0 = None
        self._gyro_bias_accum = [0.0, 0.0, 0.0]
        self._gyro_bias_count = 0
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._gyro_bias_finalized = False

        # prev timestamps for integration
        self._gyro_prev_ts = None
        self._compl_prev_ts = None

        # complementary filter weight (weight on gyro-integrated angle)
        # This is the default we tested and recommend
        self.alpha_cf = 0.98

        # bias window in seconds (first stationary period used to estimate gyro bias)
        self._bias_window = 2.0

        self.dataReader = dataReader

    # -----------------------------
    # processAccel
    # -----------------------------
    def processAccel(self, timestamp, accel_vec):
        """
        Compute roll (alpha) and pitch (beta) from accelerometer vector.
        Returns (alpha_deg, beta_deg, 0.0)
        Numerically stable formulas:
            roll  = atan2(ay, az)
            pitch = atan2(-ax, sqrt(ay^2 + az^2))
        Normalizes accelerometer vector before use.
        """
        try:
            ax, ay, az = accel_vec
        except:
            return (0.0, 0.0, 0.0)

        mag = math.sqrt(ax*ax + ay*ay + az*az)
        if mag < 1e-9:
            return (0.0, 0.0, 0.0)
        axn, ayn, azn = ax/mag, ay/mag, az/mag

        roll  = math.atan2(ayn, azn)
        pitch = math.atan2(-axn, math.sqrt(ayn*ayn + azn*azn))

        return (roll * 180.0/math.pi, pitch * 180.0/math.pi, 0.0)

    # -----------------------------
    # processGyro
    # -----------------------------
    def processGyro(self, timestamp, gyro_vec):
        """
        Integrate gyroscope rates to produce angles.
        - gyro_vec: [gx, gy, gz] in rad/s
        Returns (alpha_deg, beta_deg, 0.0)
        Uses simple Euler integration: angle += omega * dt
        Bias is estimated over the initial self._bias_window seconds.
        """
        gx, gy, gz = gyro_vec[0], gyro_vec[1], gyro_vec[2]

        # initialize t0 if needed
        if self._gyro_t0 is None:
            self._gyro_t0 = timestamp
            # prev_ts set to current sample so first dt = 0.0; next step will produce dt > 0
            self._gyro_prev_ts = timestamp

        # accumulate bias in initial window
        if (not self._gyro_bias_finalized) and (timestamp <= self._gyro_t0 + self._bias_window):
            self._gyro_bias_accum[0] += gx
            self._gyro_bias_accum[1] += gy
            self._gyro_bias_accum[2] += gz
            self._gyro_bias_count += 1
            bias_to_sub = (0.0, 0.0, 0.0)
        elif (not self._gyro_bias_finalized):
            # finalize bias average
            if self._gyro_bias_count > 0:
                self._gyro_bias[0] = self._gyro_bias_accum[0] / self._gyro_bias_count
                self._gyro_bias[1] = self._gyro_bias_accum[1] / self._gyro_bias_count
                self._gyro_bias[2] = self._gyro_bias_accum[2] / self._gyro_bias_count
            else:
                self._gyro_bias = [0.0, 0.0, 0.0]
            self._gyro_bias_finalized = True
            bias_to_sub = self._gyro_bias
        else:
            bias_to_sub = self._gyro_bias

        # compute dt
        if self._gyro_prev_ts is None:
            dt = 0.0
        else:
            dt = timestamp - self._gyro_prev_ts
            if dt < 0:
                dt = 0.0

        # subtract bias and integrate
        gx_corr = gx - bias_to_sub[0]
        gy_corr = gy - bias_to_sub[1]

        self.alphaGInt += gx_corr * dt
        self.betaGInt  += gy_corr * dt
        self._gyro_prev_ts = timestamp

        return (self.alphaGInt * 180.0/math.pi, self.betaGInt * 180.0/math.pi, 0.0)

    # -----------------------------
    # processCompl
    # -----------------------------
    def processCompl(self, timestamp, accel_vec, gyro_vec):
        """
        Complementary filter combining accel-derived angles and gyro-integrated angles.
        - accel_vec: [ax, ay, az]
        - gyro_vec:  [gx, gy, gz] in rad/s
        Returns (alpha_deg, beta_deg, 0.0)
        """
        a_alpha_deg, a_beta_deg, _ = self.processAccel(timestamp, accel_vec)

        # ensure bias t0 exists
        if self._gyro_t0 is None:
            self._gyro_t0 = timestamp
        if self._compl_prev_ts is None:
            self._compl_prev_ts = timestamp

        # choose bias (only subtract after finalized)
        if (not self._gyro_bias_finalized) and (timestamp <= self._gyro_t0 + self._bias_window):
            bias_to_sub = (0.0, 0.0, 0.0)
        else:
            bias_to_sub = self._gyro_bias

        gx = gyro_vec[0] - bias_to_sub[0]
        gy = gyro_vec[1] - bias_to_sub[1]

        # dt for complementary integrator
        dt = timestamp - self._compl_prev_ts
        if dt < 0:
            dt = 0.0

        # integrate gyro branch
        self.alphaCInt += gx * dt
        self.betaCInt  += gy * dt
        self._compl_prev_ts = timestamp

        gyro_alpha_deg = self.alphaCInt * 180.0/math.pi
        gyro_beta_deg  = self.betaCInt  * 180.0/math.pi

        k = getattr(self, "alpha_cf", 0.98)
        out_alpha = k * gyro_alpha_deg + (1.0 - k) * a_alpha_deg
        out_beta  = k * gyro_beta_deg  + (1.0 - k) * a_beta_deg

        return (out_alpha, out_beta, 0.0)

    # -----------------------------
    # GUI helpers & run loop
    # -----------------------------
    def createWindow(self):
        import dearpygui.dearpygui as dpg
        with dpg.window(tag="Status"):
            with dpg.tab_bar():
                with dpg.tab(label="Accel"):
                    self.accelPlot.createGUI(-1, -1)
                with dpg.tab(label="Gyro"):
                    self.gyroPlot.createGUI(-1, -1)
                with dpg.tab(label="Accel Result"):
                    self.accelResPlot.createGUI(-1, -1)
                with dpg.tab(label="Gyro Result"):
                    self.gyroResPlot.createGUI(-1, -1)
                with dpg.tab(label="Compl Result"):
                    self.complResPlot.createGUI(-1, -1)

    def run(self):
        import dearpygui.dearpygui as dpg
        dpg.create_context()
        dpg.create_viewport()
        self.createWindow()
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("Status", True)

        print("Waiting for data...")
        while dpg.is_dearpygui_running():
            while True:
                data = self.dataReader.readDataLine(7)
                if data is None:
                    dpg.render_dearpygui_frame()
                    break
                t = data[0]
                accel = data[1:4]
                # convert gyro from deg/s (file) to rad/s
                gyro_rad = [v * math.pi/180.0 for v in data[4:7]]

                self.accelPlot.addDataVector(t, accel)
                # show raw gyro (deg/s) for user visualization
                self.gyroPlot.addDataVector(t, data[4:7])
                self.accelResPlot.addDataVector(t, self.processAccel(t, accel))
                self.gyroResPlot.addDataVector(t, self.processGyro(t, gyro_rad))
                self.complResPlot.addDataVector(t, self.processCompl(t, accel, gyro_rad))
                dpg.render_dearpygui_frame()

        dpg.destroy_context()

# -----------------------------
# Self-test
# -----------------------------
def run_self_test():
    """
    Simulate bias window then 1s of rotation to validate the filter.
    """
    print("Running IMU self-test (2s stationary + 1s 5deg/s rotation)...")
    gui = IMUGui(None)
    gui.alpha_cf = 0.98
    dt = 0.1
    t = 0.0

    # stationary for bias window
    n_stat = int(round(gui._bias_window / dt))
    for _ in range(n_stat + 1):
        accel = [0.0, 0.0, 1.0]
        gyro = [0.0, 0.0, 0.0]
        gui.processCompl(t, accel, gyro)
        t += dt

    # rotation at 5 deg/s for 1 s
    gx = 5.0 * math.pi / 180.0
    res = (0.0, 0.0, 0.0)
    n_rot = int(round(1.0 / dt))
    for _ in range(n_rot):
        accel = [0.0, 0.0, 1.0]
        gyro = [gx, 0.0, 0.0]
        res = gui.processCompl(t, accel, gyro)
        t += dt

    print("Self-test result (Compl): alpha={:.2f} deg, beta={:.2f} deg".format(res[0], res[1]))
    print("Gyro-integrated (deg): alpha={:.2f}, beta={:.2f}".format(gui.alphaCInt*180.0/math.pi, gui.betaCInt*180.0/math.pi))

# -----------------------------
# CLI entry
# -----------------------------
if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--self-test":
        run_self_test()
        sys.exit(0)

    if len(sys.argv) < 2:
        print("Usage: python imu_new.py <imudata.txt>  OR python imu_new.py --self-test")
        sys.exit(1)

    reader = DataReader(sys.argv[1])
    gui = IMUGui(reader)
    try:
        gui.run()
    except KeyboardInterrupt:
        print("Interrupted, exiting.")
        sys.exit(0)
