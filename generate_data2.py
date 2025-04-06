#### BAROMETER
# constant acceleration to about 5 m/s, then constant speed, then cutdown and start accelerating downward at 5m/s^2
# pressure random 0-100 kPa
# temperature random 0-100C

#### MPU
# constant acceleration to about 5 m/s, then constant speed, then cutdown and start accelerating downward at 5m/s^2
# xyz for acceleration calculated randomly
# gyro random between -pi/2 and pi/2
# temperature random 0-100C

#### GPS
# location random within 0.01 degrees of bell tower
# velocity calculated from time

#### Constants
# How high is cutdown? 
# Assuming a 0.01s period between baro and mpu datapoints, 0.5s period between gps datapoints

import numpy as np

# random number generator
rng = np.random.default_rng()

# Generate a random unit vector
X_VEC = rng.random(size=3) * 2 - 1
X_VEC /= np.linalg.norm(X_VEC)

# Generate a second vector and make it perpendicular to the first
temp_vec = rng.random(size=3) * 2 - 1
Y_VEC = temp_vec - np.dot(temp_vec, X_VEC) * X_VEC
Y_VEC /= np.linalg.norm(Y_VEC)

# Generate a third vector perpendicular to both
Z_VEC = np.cross(X_VEC, Y_VEC)
Z_VEC /= np.linalg.norm(Z_VEC)

GPS_CENTER = (40.4343, -86.9488)  # Bell Tower coordinates

class BaroData:
    def __init__(self, alt):
        self.alt = alt
        self.pressure = rng.random() * 100
        self.temperature = rng.random() * 100

    def __str__(self):
        return f"{self.temperature:.1f}, {self.pressure:.2f}, {self.alt:.2f}"

class MpuData:
    def __init__(self, accel):
        ac_up = [0, 0, accel]
        acx = np.dot(ac_up, X_VEC) / np.linalg.norm(X_VEC)
        acy = np.dot(ac_up, Y_VEC) / np.linalg.norm(Y_VEC)
        acz = np.dot(ac_up, Z_VEC) / np.linalg.norm(Z_VEC)
        self.accel = [acx, acy, acz]
        self.gyro = [rng.normal() * np.pi - np.pi / 2, rng.normal() * np.pi - np.pi / 2, rng.normal() * np.pi - np.pi / 2]
        self.temperature = rng.random() * 100

    def __str__(self):
        return f"{self.temperature:.1f}, {self.accel[0]:.3f}, {self.accel[1]:.3f}, {self.accel[2]:.3f}, {self.gyro[0]:.3f}, {self.gyro[1]:.3f}, {self.gyro[2]:.3f}"

class GpsData:
    def __init__(self, t, alt, prev_lat, prev_lon, prev_t):
        self.time = t
        self.lat = GPS_CENTER[0] + rng.random() * 0.0002 - 0.0001
        self.lon = GPS_CENTER[1] + rng.random() * 0.0002 - 0.0001
        self.alt = alt

        if abs(prev_t - t) <= 1e-5:
            self.vel = 0.0
            self.heading = 0.0
        else:
            temp_vel_part = np.array([self.lat - prev_lat, self.lon - prev_lon])
            self.vel = np.linalg.norm(temp_vel_part) / (t - prev_t) * 111200
            self.heading = np.angle(complex(temp_vel_part[0], temp_vel_part[1]), True)

    def __str__(self):
        return f"{self.lat:.5f}, {self.lon:.5f}, {self.alt:.2f}, {self.vel:.3}, {self.heading:.3f}, {self.time:.2f}"

time = np.linspace(0, 200, 20001)
upward_accel = np.zeros(20001)
upward_vel = np.zeros(20001)
alt = np.zeros(20001)

baro_file, mpu_file, gps_file = (open("baro2.csv", "w"), open("mpu2.csv", "w"), open("gps2.csv", "w"))
    
prev_gps = None

# first stage of trajectory: ascent acceleration
for i, t in enumerate(time[:300]):

    # gps prev data

    # trajectory 
    upward_accel[i] = 5
    upward_vel[i] = 5 * t
    alt[i] = 0.5 * 5 * t ** 2
    print(f"i: {i}, t: {t}, alt: {alt[i]}, upward_accel: {upward_accel[i]}, upward_vel: {upward_vel[i]}")


    baro_file.write(str(BaroData(alt[i])) + "\n")
    mpu_file.write(str(MpuData(upward_accel[i])) + "\n")

    if (i % 50 == 0):
        if prev_gps is None:
            prev_gps = GpsData(t, alt[i], GPS_CENTER[0], GPS_CENTER[1], 0)
            gps_file.write(str(prev_gps) + "\n")
        else:
            prev_gps = GpsData(t, alt[i], prev_gps.lat, prev_gps.lon, prev_gps.time)
            gps_file.write(str(prev_gps) + "\n")
            

# second stage of trajectory: ascent terminal velocity
t0 = time[i]
v0 = upward_vel[i]
h0 = alt[i]
while alt[i] < 2000: # 1600 meters = ~5000 ft
    i += 1

    t = time[i]
    upward_accel[i] = 0
    upward_vel[i] = v0
    alt[i] = v0 * (t - t0) + h0
    if i < 200:
        print(f"i: {i}, t: {t}, alt: {alt[i]}, upward_accel: {upward_accel[i]}, upward_vel: {upward_vel[i]}")

    baro_file.write(str(BaroData(alt[i])) + "\n")
    mpu_file.write(str(MpuData(upward_accel[i])) + "\n")

    if (i % 50 == 0):
        prev_gps = GpsData(t, alt[i], prev_gps.lat, prev_gps.lon, prev_gps.time)
        gps_file.write(str(prev_gps) + "\n")

i_f = i

baro_file.close()
mpu_file.close()
gps_file.close()

import matplotlib.pyplot as plt

print(f"Final index: {i_f}")

# Plotting the data
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# Upward acceleration
axs[0].plot(time[:i_f], upward_accel[:i_f], label="Upward Acceleration")
axs[0].set_ylabel("Acceleration (m/s²)")
axs[0].legend()
axs[0].grid()

# Upward velocity
axs[1].plot(time[:i_f], upward_vel[:i_f], label="Upward Velocity", color="orange")
axs[1].set_ylabel("Velocity (m/s)")
axs[1].legend()
axs[1].grid()

# Altitude
axs[2].plot(time[:i_f], alt[:i_f], label="Altitude", color="green")
axs[2].set_xlabel("Time (s)")
axs[2].set_ylabel("Altitude (m)")
axs[2].legend()
axs[2].grid()

plt.tight_layout()
plt.show()