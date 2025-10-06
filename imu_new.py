# imu.py (full patched version)
# Implements processAccel, processGyro, processCompl with bias estimation,
# deg/s -> rad/s conversion in run(), and a built-in self-test (--self-test).
#
# Outputs from processing functions are angles in DEGREES: (alpha, beta, 0)
# alpha = roll about X, beta = pitch about Y
#
# Save as imu.py (or imu_new.py) and run:
#   python imu.py imudata.txt
# or run the built-in self-test:
#   python imu.py --self-test

import dearpygui.dearpygui as dpg
import sys
import select
import time
import math
from DataPlot import DataPlot

class DataReader:
    """
    Reads lines of comma separated floats from a file or device.
    Expects lines: timestamp, ax, ay, az, gx, gy, gz
    """
    def __init__(self, filename: str):
        self.file = open(filename, "r")
        self.dataFromDevice = filename.startswith("/dev/")
        self.lastTimestamp = -1

    def readDataLine(self, floatcount: int = -1) -> list:
        if self.dataFromDevice == True:
            r, w, x = select.select([self.file], [], [], 0)
            if len(r) == 0:
                return None
        line = self.file.readline()
        if line == "":
            return None
        vector = line.split(",")
        if floatcount != -1 and len(vector) != floatcount:
            return None
        try:
            vector = list(map(float, vector))
        except:
            return None
        if self.dataFromDevice == False:
            if self.lastTimestamp != -1:
                # simulate real-time playback
                time.sleep(max(0.0, vector[0] - self.lastTimestamp))
        self.lastTimestamp = vector[0]
        return vector

class IMUGui:
    def __init__(self, dataReader):
        # plots
        self.accelPlot = DataPlot(("ax", "ay", "az"), 1000)
        self.gyroPlot = DataPlot(("p", "q", "r"), 1000)
        self.accelResPlot = DataPlot(("alpha", "beta", "gamma"), 1000)
        self.gyroResPlot = DataPlot(("alpha", "beta", "gamma"), 1000)
        self.complResPlot = DataPlot(("alpha", "beta", "gamma"), 1000)

        # integration states (radians internally)
        self.alphaGInt = 0.0
        self.betaGInt = 0.0
        self.alphaCInt = 0.0
        self.betaCInt = 0.0

        # gyro bias estimation state
        self._gyro_t0 = None
        self._gyro_bias_accum = [0.0, 0.0, 0.0]
        self._gyro_bias_count = 0
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._gyro_bias_finalized = False

        # previous timestamps for integration
        self._gyro_prev_ts = None
        self._compl_prev_ts = None

        # complementary filter parameter (weight on gyro-integrated angle)
        self.alpha_cf = 0.98  # default: trust gyro short-term heavily

        # data reader
        self.dataReader = dataReader

    def createWindow(self):
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

    def processAccel(self, timestamp, accel):
        """
        Compute roll (alpha) and pitch (beta) from accelerometer.
        - accel: [ax, ay, az] (units proportional to gravity)
        Returns (alpha_deg, beta_deg, 0.0)
        """
        try:
            ax, ay, az = accel
        except:
            return (0.0, 0.0, 0.0)

        mag = math.sqrt(ax*ax + ay*ay + az*az)
        if mag < 1e-6:
            return (0.0, 0.0, 0.0)
        axn, ayn, azn = ax/mag, ay/mag, az/mag

        # numerically stable formulas
        roll = math.atan2(ayn, azn)
        pitch = math.atan2(-axn, math.sqrt(ayn*ayn + azn*azn))

        alpha_deg = roll * (180.0 / math.pi)
        beta_deg = pitch * (180.0 / math.pi)
        return (alpha_deg, beta_deg, 0.0)

    def processGyro(self, timestamp, gyro):
        """
        Integrate gyro rates to angles.
        - gyro: [gx, gy, gz] in radians/sec (must convert deg/s -> rad/s before calling if needed)
        Returns (alpha_deg, beta_deg, 0.0)
        Bias averaged over initial calibration window (2.0s here).
        """
        gx, gy, gz = gyro

        # initialize bias window start
        if self._gyro_t0 is None:
            self._gyro_t0 = timestamp
            self._gyro_prev_ts = timestamp
            self.alphaGInt = getattr(self, "alphaGInt", 0.0)
            self.betaGInt = getattr(self, "betaGInt", 0.0)

        # bias accumulation window: first 2.0 seconds after _gyro_t0
        if (not self._gyro_bias_finalized) and (timestamp <= self._gyro_t0 + 2.0):
            self._gyro_bias_accum[0] += gx
            self._gyro_bias_accum[1] += gy
            self._gyro_bias_accum[2] += gz
            self._gyro_bias_count += 1
            bias_to_sub = [0.0, 0.0, 0.0]
        elif (not self._gyro_bias_finalized):
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

        # subtract bias and integrate (Euler)
        gx_corr = gx - bias_to_sub[0]
        gy_corr = gy - bias_to_sub[1]

        if self._gyro_prev_ts is None:
            dt = 0.0
        else:
            dt = timestamp - self._gyro_prev_ts
            if dt < 0:
                dt = 0.0

        self.alphaGInt += gx_corr * dt
        self.betaGInt += gy_corr * dt
        self._gyro_prev_ts = timestamp

        alpha_deg = self.alphaGInt * (180.0 / math.pi)
        beta_deg = self.betaGInt * (180.0 / math.pi)
        return (alpha_deg, beta_deg, 0.0)

    def processCompl(self, timestamp, accel, gyro):
        """
        Complementary filter combining accel-derived angles and gyro-integrated angles.
        - accel: [ax, ay, az]
        - gyro: [gx, gy, gz] in radians/sec
        Returns (alpha_deg, beta_deg, 0.0)
        """
        accel_alpha_deg, accel_beta_deg, _ = self.processAccel(timestamp, accel)

        # ensure bias init exists
        if self._gyro_t0 is None:
            self._gyro_t0 = timestamp
            self._gyro_prev_ts = timestamp
            self._compl_prev_ts = timestamp
            self.alphaCInt = getattr(self, "alphaCInt", 0.0)
            self.betaCInt = getattr(self, "betaCInt", 0.0)

        # decide bias to subtract (same 2.0s window)
        if (not self._gyro_bias_finalized) and (timestamp <= self._gyro_t0 + 2.0):
            bias_to_sub = [0.0, 0.0, 0.0]
        else:
            bias_to_sub = self._gyro_bias

        gx, gy, gz = gyro[0] - bias_to_sub[0], gyro[1] - bias_to_sub[1], gyro[2] - bias_to_sub[2]

        if self._compl_prev_ts is None:
            dt = 0.0
        else:
            dt = timestamp - self._compl_prev_ts
            if dt < 0:
                dt = 0.0

        # integrate gyro branch for complementary filter
        self.alphaCInt += gx * dt
        self.betaCInt += gy * dt
        self._compl_prev_ts = timestamp

        gyro_alpha_deg = self.alphaCInt * (180.0 / math.pi)
        gyro_beta_deg = self.betaCInt * (180.0 / math.pi)

        k = getattr(self, "alpha_cf", 0.98)
        out_alpha = k * gyro_alpha_deg + (1.0 - k) * accel_alpha_deg
        out_beta = k * gyro_beta_deg + (1.0 - k) * accel_beta_deg

        return (out_alpha, out_beta, 0.0)

    def run(self):
        dpg.create_context()
        dpg.create_viewport()
        self.createWindow()
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("Status", True)
        print("Waiting for data.")
        while dpg.is_dearpygui_running():
            while True:
                data = self.dataReader.readDataLine(7)
                if data is None:
                    dpg.render_dearpygui_frame()
                    break
                # data format: [timestamp, ax, ay, az, gx, gy, gz]
                self.accelPlot.addDataVector(data[0], data[1:4])
                # show raw gyro rates in deg/s for visualization
                self.gyroPlot.addDataVector(data[0], [(180.0/math.pi)*v for v in data[4:7]])
                self.accelResPlot.addDataVector(data[0], self.processAccel(data[0], data[1:4]))

                # convert gyro deg/s -> rad/s (your input file appears to be deg/s)
                gyro_vec = [v * math.pi / 180.0 for v in data[4:7]]

                self.gyroResPlot.addDataVector(data[0], self.processGyro(data[0], gyro_vec))
                self.complResPlot.addDataVector(data[0], self.processCompl(data[0], data[1:4], gyro_vec))
                dpg.render_dearpygui_frame()
        dpg.destroy_context()

# ----------------------------------------------------------
# Optional self-test: simulate bias + small rotation
# Run via: python imu.py --self-test
# ----------------------------------------------------------
def run_self_test():
    import types
    print("\nRunning IMU self-test...")
    d = type("D", (), {})()
    # minimal attributes used by methods
    d._gyro_t0 = None
    d._gyro_bias_accum = [0.0, 0.0, 0.0]
    d._gyro_bias_count = 0
    d._gyro_bias = [0.0, 0.0, 0.0]
    d._gyro_bias_finalized = False
    d._gyro_prev_ts = None
    d._compl_prev_ts = None
    d.alpha_cf = 0.98
    d.alphaGInt = d.betaGInt = d.alphaCInt = d.betaCInt = 0.0

    # bind IMUGui methods to dummy
    d.processAccel = types.MethodType(IMUGui.processAccel, d)
    d.processGyro  = types.MethodType(IMUGui.processGyro, d)
    d.processCompl = types.MethodType(IMUGui.processCompl, d)

    t, dt = 0.0, 0.1
    # simulate 2.0s stationary (bias window set to 2.0)
    for _ in range(20):
        a = [0.0, 0.0, 1.0]
        g = [0.0, 0.0, 0.0]
        d.processCompl(t, a, g)
        t += dt

    # now apply 5 deg/s rotation around X for 1s
    gx = 5.0 * math.pi / 180.0
    res = (0.0, 0.0, 0.0)
    for _ in range(10):
        a = [0.0, 0.0, 1.0]
        g = [gx, 0.0, 0.0]
        res = d.processCompl(t, a, g)
        t += dt
    print(f"Expected ≈5°, got α={res[0]:.2f}°, β={res[1]:.2f}°\n")

if __name__ == "__main__":
    # self-test mode
    if len(sys.argv) > 1 and sys.argv[1] == "--self-test":
        run_self_test()
        sys.exit(0)

    if len(sys.argv) < 2:
        print("Usage: %s <file_or_port> OR python imu.py --self-test" % sys.argv[0])
        sys.exit(-1)
    reader = DataReader(sys.argv[1])
    gui = IMUGui(reader)
    gui.run()
