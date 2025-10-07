# 🛰️ Sensor Networks and Sensor Data Fusion  
##  Accelerometer/Gyro Fusion  

### 🎯 Objective  
This exercise implements **IMU sensor fusion** using an **accelerometer** and **gyroscope**.  
The goal is to estimate the **orientation (roll and pitch)** of an IMU board by combining both sensors —  
the accelerometer gives a stable but noisy estimate of tilt, while the gyroscope gives a smooth but drifting one.  
The result is a **complementary filter** that fuses both signals to get a clean, stable angle estimate.  

---

### 📁 Files Overview  

| File | Description |
|------|--------------|
| **`imu.py`** | Main program containing the core IMU processing logic. |
| **`DataPlot.py`** | Visualization helper using DearPyGui for plotting sensor data and results. |
| **`imudata.txt`** | Recorded IMU dataset used for testing and visualization. |
| **`tune_alpha.py`** | Helper script to tune the complementary filter parameter `alpha_cf`. |
| **`tune_alpha_debug*.py`** | Debugging scripts used during development to validate bias handling and gyro integration. |
| **`show_gyro_source.py`** | Script to verify which IMU source file and function definitions are being used. |

---

### 🧠 Implemented Functions  

All three key functions were implemented in **`imu.py`**:

#### `processAccel(timestamp, accel_vec)`  
- Computes **roll (α)** and **pitch (β)** using the direction of gravity.  
- Formulae used:  
  \[
  \alpha = \arctan\left(\frac{a_y}{a_z}\right), \quad 
  \beta = \arctan\left(\frac{-a_x}{\sqrt{a_y^2 + a_z^2}}\right)
  \]
- Returns the result in **degrees**.  
- Stable long-term orientation reference, though slightly noisy during movement.  

#### `processGyro(timestamp, gyro_vec)`  
- Integrates **angular velocity** over time to estimate **roll** and **pitch**.  
- Implements **bias estimation** during the first ~2 seconds of stationary data.  
- Subtracts bias before integration to prevent drift.  
- Uses Euler integration and returns angles in **degrees**.  
- Provides smooth, short-term orientation but drifts slowly if used alone.  

#### `processCompl(timestamp, accel_vec, gyro_vec)`  
- Implements a **complementary filter** that fuses the two signals:  
  \[
  \text{angle} = \alpha_{cf} \times \text{gyro\_angle} + (1 - \alpha_{cf}) \times \text{accel\_angle}
  \]
- Parameter `alpha_cf = 0.98` chosen after tuning (best trade-off between smoothness and stability).  
- Produces final, fused roll and pitch angles.  

---

### ⚙️ Complementary Filter Behavior  

| Sensor | Advantage | Drawback |
|---------|------------|----------|
| **Accelerometer** | Long-term stability (gravity reference) | Noisy when moving |
| **Gyroscope** | Smooth short-term motion tracking | Drifts over time |
| **Fusion Result (Compl)** | Combines both: smooth *and* stable | ✅ Best overall |

This fusion algorithm is conceptually simple but forms the **foundation of modern AHRS (Attitude and Heading Reference Systems)** used in drones, robotics, and mobile devices.

---

### 🧪 Recorded Dataset  

The dataset (`imudata.txt`) is ~30 seconds long and includes:
- ~3 seconds stationary for bias calibration  
- Lateral movements in ±X, ±Y, ±Z  
- Rotations around X and Y axes in both directions  

Each line contains seven comma-separated floats:  
```
timestamp, ax, ay, az, gx, gy, gz
```
where:
- `a*` = acceleration (m/s² or g)  
- `g*` = angular velocity (rad/s)

---

### 🧩 Results and Observations  

#### ✅ Before Implementation  
- **Gyro Result** and **Compl Result** plots were flat (no orientation calculated).  
- Only raw accel and gyro signals visible.  
- No meaningful roll/pitch output.

#### ✅ After Implementation  
- **Accel Result:** Gravity-based roll/pitch visible, but noisy.  
- **Gyro Result:** Smooth angle curves, showing rotation motion.  
- **Compl Result:** Stable, drift-free orientation output — smooth transitions and accurate reset to zero after motion.  

> Gamma (yaw) remains flat because yaw estimation requires a **magnetometer** or **Z-axis gyro integration** (not implemented in this exercise).  

---

### ⚖️ Filter Parameter Tuning  

Using `tune_alpha.py`, the complementary filter weight `alpha_cf` was tested for values 0.90 → 0.995.  
Results showed:

| α_cf | Fused α (deg) | Error vs Expected (5° rotation) |
|------|----------------|--------------------------------|
| 0.90 | 4.50 | -0.50 |
| 0.95 | 4.75 | -0.25 |
| 0.98 | **4.90** | **-0.10** ✅ |
| 0.99 | 4.95 | -0.05 |
| 0.995 | 4.97 | -0.03 |

**Chosen value:** `alpha_cf = 0.98`

---

### 🖥️ Visualization  

Run the program to visualize all signals and results:  
```bash
python imu.py imudata.txt
```

#### Tabs in the GUI:
- **Accel:** raw accelerometer data  
- **Gyro:** raw gyroscope data  
- **Accel Result:** roll/pitch from accelerometer  
- **Gyro Result:** roll/pitch from gyroscope  
- **Compl Result:** fused complementary filter output  

---

### 🔍 Interpretation of Graphs  

| Plot | Description | Meaning |
|------|--------------|---------|
| **Accel** | Raw acceleration (ax, ay, az) | Shows gravity and motion noise |
| **Gyro** | Angular velocities (p, q, r) | Smooth short-term rotations |
| **Accel Result** | Tilt from gravity | Stable but noisy |
| **Gyro Result** | Integrated tilt | Smooth but drifty |
| **Compl Result** | Fused angles | Smooth, stable, drift-free ✅ |

---

### 💡 Key Takeaways  

- The complementary filter combines accelerometer and gyroscope readings to produce a **clean, stable, and accurate** estimate of orientation.  
- The **gyro bias correction** phase eliminates drift.  
- The **α=0.98** weighting gives near-real-time response while maintaining long-term stability.  
- The resulting filter mimics the core behavior of an **AHRS**, used in drones, robotics, and smartphones.

---

### 🚀 Future Extensions  

You can extend this project by:
1. Adding **yaw (γ)** estimation using gyro Z-axis and magnetometer data.  
2. Implementing a **Kalman filter** or **Madgwick filter** for full 9-DOF fusion.  
3. Running the code on a **live IMU** (e.g., MPU6050) to visualize real-time orientation.  

---

### 👨‍💻 Author  
**Ashwin AIAS**  
Sensor Networks and Sensor Data Fusion — IMU Complementary Filter Implementation  
Technische Hochschule Ingolstadt  
