# Sensor Networks and Sensor Data Fusion
## Programming Exercise 1: Accelerometer/Gyro Fusion

### Description
#### Files
* `imu.py` is your main program. 
* `DataPlot.py` is a helper class for visualization.
* `imudata.txt` contains recorded data you can use -- description below.

You can run the program using `python imu.py imudata.txt` after setting up your work environment. See below.

I have left the functions `processAccel()`, `processGyro()` and `processCompl()` as exercise for you. They all take a timestamp
and a 3-vector with accelerometer and/or gyro data measured at that timestamp, respectively. They should all return a 3-vector
(alpha, beta, 0) that then gets plotted. Please use the information you learned in the lecture to fill the three functions to compute
* rotation obtained from pure accelerometer data (by looking at the direction of the down vector given by gravity)
* rotation obtained from pure gyro data (by integrating turn rates)
* rotation obtained by fusing accelerometer and gyro data with a complementary filter.
If you have questions or comments chat me up in Teams or write an email to bjoern.giesler@thi.de!

### Set up your work environment 
To prepare your work environment, you need
* Python 3.10 or later -- www.python.org
* The DearPyGui user interface library. Install with "pip install dearpygui" (pip comes with the Python3.x you just installed).
* Nothing else should be required. Let me know if you can't get it to work.

### Description of the recorded data
The data you see is not very exact. It is approximately 30s long and was obtained by
* waiting for approximately 3 seconds after start with no motion
* moving the IMU board laterally with about 1s delay:
    * in positive x
    * in negative x
    * in positive y
    * in negative y
    * in positive z
    * in negative z
* turning the IMU board
	* in positive direction around x, then back to zero
	* in negative direction around x, then back to zero
	* in positive direction around y, then back to zero
	* in negative direction around y, then back to zero
The format is very simple - seven floats separated by comma. The first float is a timestamp, the next three are accelerometer information (x, y, z axis), the last three are gyro information (rotation around x, y, z axis).

### Some usage hints
* The Graph visualization is quite powerful. 
* Double clicking auto-scales the view to show all data points.
* Pan vertically by clicking and dragging with the left mouse button.
* Zoom in or out with the mouse wheel.
* Drag with the right mouse button to zoom into a specific vertical region.
* Click individual curves in the legend (top left corner) to enable/disable.
