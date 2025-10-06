# imu.py (patched – bias window = 2.0s)
import dearpygui.dearpygui as dpg
import sys
import select
import time
import math
import sys
from DataPlot import DataPlot

class DataReader:
	def __init__(self, filename: str):
		self.file = open(filename, "r")
		self.dataFromDevice = filename.startswith("/dev/")
		self.lastTimestamp = -1

	def readDataLine(self,  floatcount: int = -1) -> list:
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
				time.sleep(vector[0]-self.lastTimestamp)
		self.lastTimestamp = vector[0]	
		return vector

class IMUGui:
	def __init__(self, dataReader):
		self.accelPlot = DataPlot(("ax", "ay", "az"), 1000)
		self.gyroPlot = DataPlot(("p", "q", "r"), 1000)
		self.accelResPlot = DataPlot(("alpha", "beta", "gamma"), 1000)
		self.gyroResPlot = DataPlot(("alpha", "beta", "gamma"), 1000)
		self.complResPlot = DataPlot(("alpha", "beta", "gamma"), 1000)

		self.lastTimestamp = -1
		self.alphaGInt = 0.0
		self.betaGInt = 0.0
		self.alphaCInt = 0.0
		self.betaCInt = 0.0

		self.complK = 0.05

		# --- gyro bias estimation ---
		self._gyro_t0 = None
		self._gyro_bias_accum = [0.0, 0.0, 0.0]
		self._gyro_bias_count = 0
		self._gyro_bias = [0.0, 0.0, 0.0]
		self._gyro_bias_finalized = False
		self._gyro_prev_ts = None
		self._compl_prev_ts = None

		self.alpha_cf = 0.98  # complementary filter weight
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
		try:
			ax, ay, az = accel
		except:
			return (0.0, 0.0, 0.0)
		mag = math.sqrt(ax*ax + ay*ay + az*az)
		if mag < 1e-6:
			return (0.0, 0.0, 0.0)
		axn, ayn, azn = ax/mag, ay/mag, az/mag
		roll = math.atan2(ayn, azn)
		pitch = math.atan2(-axn, math.sqrt(ayn*ayn + azn*azn))
		alpha_deg = roll * (180.0 / math.pi)
		beta_deg = pitch * (180.0 / math.pi)
		return (alpha_deg, beta_deg, 0.0)

	def processGyro(self, timestamp, gyro):
		gx, gy, gz = gyro
		if self._gyro_t0 is None:
			self._gyro_t0 = timestamp
			self._gyro_prev_ts = timestamp
			self.alphaGInt = getattr(self, "alphaGInt", 0.0)
			self.betaGInt = getattr(self, "betaGInt", 0.0)

		# bias accumulation (2.0s window)
		if (not self._gyro_bias_finalized) and (timestamp <= self._gyro_t0 + 2.0):
			self._gyro_bias_accum[0] += gx
			self._gyro_bias_accum[1] += gy
			self._gyro_bias_accum[2] += gz
			self._gyro_bias_count += 1
			bias_to_sub = [0.0, 0.0, 0.0]
		elif (not self._gyro_bias_finalized):
			if self._gyro_bias_count > 0:
				self._gyro_bias = [
					self._gyro_bias_accum[0] / self._gyro_bias_count,
					self._gyro_bias_accum[1] / self._gyro_bias_count,
					self._gyro_bias_accum[2] / self._gyro_bias_count,
				]
			self._gyro_bias_finalized = True
			bias_to_sub = self._gyro_bias
		else:
			bias_to_sub = self._gyro_bias

		gx_corr = gx - bias_to_sub[0]
		gy_corr = gy - bias_to_sub[1]

		dt = 0.0 if self._gyro_prev_ts is None else max(timestamp - self._gyro_prev_ts, 0.0)
		self.alphaGInt += gx_corr * dt
		self.betaGInt += gy_corr * dt
		self._gyro_prev_ts = timestamp

		alpha_deg = self.alphaGInt * (180.0 / math.pi)
		beta_deg = self.betaGInt * (180.0 / math.pi)
		return (alpha_deg, beta_deg, 0.0)

	def processCompl(self, timestamp, accel, gyro):
		accel_alpha_deg, accel_beta_deg, _ = self.processAccel(timestamp, accel)

		if self._gyro_t0 is None:
			self._gyro_t0 = timestamp
			self._gyro_prev_ts = timestamp
			self._compl_prev_ts = timestamp
			self.alphaCInt = getattr(self, "alphaCInt", 0.0)
			self.betaCInt = getattr(self, "betaCInt", 0.0)

		# bias accumulation (same 2.0s window)
		if (not self._gyro_bias_finalized) and (timestamp <= self._gyro_t0 + 2.0):
			bias_to_sub = [0.0, 0.0, 0.0]
		else:
			bias_to_sub = self._gyro_bias

		gx, gy, gz = [gyro[i] - bias_to_sub[i] for i in range(3)]
		dt = 0.0 if self._compl_prev_ts is None else max(timestamp - self._compl_prev_ts, 0.0)
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
				self.accelPlot.addDataVector(data[0], data[1:4])
				self.gyroPlot.addDataVector(data[0], [(180.0/math.pi)*v for v in data[4:7]])
				self.accelResPlot.addDataVector(data[0], self.processAccel(data[0], data[1:4]))

				# convert gyro deg/s → rad/s
				gyro_vec = [v * math.pi / 180.0 for v in data[4:7]]
				self.gyroResPlot.addDataVector(data[0], self.processGyro(data[0], gyro_vec))
				self.complResPlot.addDataVector(data[0], self.processCompl(data[0], data[1:4], gyro_vec))
				dpg.render_dearpygui_frame()
		dpg.destroy_context()

if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("Usage: %s <file_or_port>" % sys.argv[0])
		sys.exit(-1)
	reader = DataReader(sys.argv[1])
	gui = IMUGui(reader)
	gui.run()
