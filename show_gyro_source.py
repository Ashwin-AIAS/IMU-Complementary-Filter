# show_gyro_source.py
import imu, inspect
print("imu module file:", getattr(imu, "__file__", "NO FILE"))
print("---- processGyro source (first 40 lines) ----")
try:
    src = inspect.getsource(imu.IMUGui.processGyro)
    for i, line in enumerate(src.splitlines()[:40], 1):
        print(f"{i:02d}: {line}")
except Exception as e:
    print("couldn't get source:", e)
