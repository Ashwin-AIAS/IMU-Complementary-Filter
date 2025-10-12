import dearpygui.dearpygui as dpg
import sys
import select
import time
import math
import sys

from DataPlot import DataPlot

class DataReader:
	"""
	Reads lines of comma separated floats from a file or device.
	"""
	def __init__(self, filename: str):
		self.file = open(filename, "r")
		self.dataFromDevice = filename.startswith("/dev/")
		self.lastTimestamp = -1

	def readDataLine(self,  floatcount: int = -1) -> list:
		"""
		If floatcount != -1, it will accept and return any number of floats.
		Otherwise it will return None if the line doesn't have exactly 
		floatcount floats.
		If the file is a device (starts with "/dev/"), we assume real-time
		delivery of the data. If not, we interpret the first float as the
		timestamp and sleep for the delay to simulate recording-time behavior.
		"""
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
		# Create all data plots to hold 1000 points before scrolling
		self.accelPlot = DataPlot(("ax", "ay", "az"), 1000)
		self.gyroPlot = DataPlot(("p", "q", "r"), 1000)
		self.accelResPlot = DataPlot(("alpha", "beta", "gamma"), 1000)
		self.gyroResPlot = DataPlot(("alpha", "beta", "gamma"), 1000)
		self.complResPlot = DataPlot(("alpha", "beta", "gamma"), 1000)

		# Initialize timestamp to show that we haven't read anything yet
		self.lastTimestamp = -1

		# Integration results from gyro (raw) and complementary filter
		self.alphaGInt = self.betaGInt = 0
		self.alphaCInt = self.betaCInt = 0

		# k constant for complementary filter
		self.complK = 0.05

		# We will use this to read data
		self.dataReader = dataReader

	def createWindow(self):
		# Build the GUI as a 
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

	def processAccel(self, timestamp, vec):
		try:
			alpha = math.atan2(vec[2], vec[1])
		except:
			alpha = 0
			print("Error processing atan2(%f,%f)" % (vec[2], vec[1]))
		try:
			beta = math.asin(vec[0]/9.8)
		except:
			beta = 0
			print("Error processing asin(%f)" % (vec[0]/9.8))
		return (alpha, beta, 0)

	def processGyro(self, timestamp, vec):
		deltaT = timestamp - self.lastTimestamp
		self.alphaGInt = self.alphaGInt + vec[0]*deltaT
		self.betaGInt = self.betaGInt + vec[1]*deltaT
		if self.alphaGInt > math.pi:
			self.alphaGInt = self.alphaGInt - math.pi
		if self.alphaGInt < -math.pi:
			self.alphaGInt = self.alphaGInt + math.pi
		if self.betaGInt > math.pi:
			self.betaGInt = self.betaGInt - math.pi
		if self.betaGInt < -math.pi:
			self.betaGInt = self.betaGInt + math.pi
		self.lasttimestamp = timestamp
		return (self.alphaGInt, self.betaGInt, 0)

	def processCompl(self, timestamp, accel, gyro):
		deltaT = timestamp - self.lastTimestamp
		accelC = self.processAccel(timestamp, accel)

		alpha = (self.alphaCInt + gyro[0]*deltaT)*(1-self.complK)
		alpha = alpha + accelC[0]*self.complK
		
		beta = (self.betaCInt + gyro[1]*deltaT)*(1-self.complK)
		beta = beta + accelC[1]*self.complK

		self.alphaCInt = alpha
		self.betaCInt = beta

		if self.alphaCInt > math.pi:
			self.alphaCInt = self.alphaCInt - math.pi
		if self.alphaCInt < -math.pi:
			self.alphaCInt = self.alphaCInt + math.pi
		if self.betaCInt > math.pi:
			self.betaCInt = self.betaCInt - math.pi
		if self.betaCInt < -math.pi:
			self.betaCInt = self.betaCInt + math.pi
		self.lastTimestamp = timestamp

		return (alpha, beta, 0)

	def run(self):
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
				self.accelPlot.addDataVector(data[0], data[1:4])
				self.gyroPlot.addDataVector(data[0], [(180.0/math.pi)*v for v in data[4:7]])
				self.accelResPlot.addDataVector(data[0], self.processAccel(data[0], data[1:4]))
				self.gyroResPlot.addDataVector(data[0], self.processGyro(data[0], data[4:7]))
				self.complResPlot.addDataVector(data[0], [(180.0/math.pi)*v for v in self.processCompl(data[0], data[1:4], data[4:7])])			
				dpg.render_dearpygui_frame()
		dpg.destroy_context()


if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("Usage: %s <file_or_port>" % sys.argv[0])
		sys.exit(-1)
	reader = DataReader(sys.argv[1])
	gui = IMUGui(reader)
	gui.run()