import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# === LOAD DATA ===
df = pd.read_csv("SerialSensorData/Ellip_test10_serial.csv")
df['PC_Timestamp'] = pd.to_datetime(df['PC_Timestamp'])
df['Elapsed_Minutes'] = (df['PC_Timestamp'] - df['PC_Timestamp'].iloc[0]).dt.total_seconds() / 60

# === COMPUTE POSITIONS IN mm ===
df['x_mm'] = (10 * (df['inputVoltageX'] - 5)) / (2 * df['inputVoltageSUM'])
df['y_mm'] = (10 * (df['inputVoltageY'] - 5)) / (2 * df['inputVoltageSUM'])

# === CALCULATE CENTER BASED ON LEVEL ===
x_center = ((10 * (df['LevelX'] - 5)) / (2 * df['inputVoltageSUM'])).mean()
y_center = ((10 * (df['LevelY'] - 5)) / (2 * df['inputVoltageSUM'])).mean()

# === CALCULATE DISTANCE FROM CENTER ===
df['distance_from_center'] = np.sqrt((df['x_mm'] - x_center)**2 + (df['y_mm'] - y_center)**2)

# === STATS ===
mean_dist = df['distance_from_center'].mean()
std_dist = df['distance_from_center'].std()
max_dist = df['distance_from_center'].max()

print("🔎 Distance from Center Stats:")
print(f"  Mean:  {mean_dist:.4f} mm")
print(f"  Std:   {std_dist:.4f} mm")
print(f"  Max:   {max_dist:.4f} mm")

# === HISTOGRAM OF DISTANCE FROM CENTER ===
plt.figure(figsize=(10, 5))
plt.hist(df['distance_from_center'], bins=50, color='steelblue', edgecolor='black')
plt.axvline(mean_dist, color='green', linestyle='--', label=f'Mean = {mean_dist:.3f} mm')
plt.axvline(0.04, color='purple', linestyle='--', label='0.04 mm Threshold')
plt.axvline(mean_dist + std_dist, color='orange', linestyle='--', label=f'+1 Std = {mean_dist + std_dist:.3f} mm')
plt.axvline(mean_dist - std_dist, color='orange', linestyle='--', label=f'-1 Std = {mean_dist - std_dist:.3f} mm')
plt.axvline(max_dist, color='red', linestyle='--', label=f'Max = {max_dist:.3f} mm')
plt.xlabel('Distance from Center (mm)')
plt.ylabel('Frequency')
plt.title('Histogram of Distance from Center')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# === COVERAGE CALCULATIONS ===
within_004mm = (df['distance_from_center'] <= 0.04).sum()
total_points = len(df)

print("📊 Histogram Coverage:")
print(f"  Within 0.04 mm: {within_004mm} points ({100 * within_004mm / total_points:.2f}%)")

# === ALIGN POSITION TO ZERO ===
df['x_aligned'] = df['x_mm'] - x_center
df['y_aligned'] = df['y_mm'] - y_center

# === ALIGNMENT PLOTS OVER TIME ===
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

ax1.plot(df['Elapsed_Minutes'], df['x_aligned'], label='X Position', color='blue', linewidth=2)
ax1.axhline(0, color='gray', linestyle='--', label='Level X = 0')
ax1.set_ylabel('Position (mm)')
ax1.set_title('X Position Relative to Level')
ax1.legend()
ax1.grid(True)

ax2.plot(df['Elapsed_Minutes'], df['y_aligned'], label='Y Position', color='green', linewidth=2)
ax2.axhline(0, color='gray', linestyle='--', label='Level Y = 0')
ax2.set_xlabel('Elapsed Time (minutes)')
ax2.set_ylabel('Position (mm)')
ax2.set_title('Y Position Relative to Level')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()

# === 2D POSITION PLOT ===
margin = 0.2
x_min, x_max = df['x_mm'].min() - margin, df['x_mm'].max() + margin
y_min, y_max = df['y_mm'].min() - margin, df['y_mm'].max() + margin

fig, ax = plt.subplots(figsize=(8, 8))
ax.plot(df['x_mm'], df['y_mm'], marker='o', markersize=2, linestyle='-', color='teal', label='Measured Position')
ax.plot(x_center, y_center, 'ro', label='Center Position')
ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
ax.set_aspect('equal')
ax.set_xlabel('x position (mm)')
ax.set_ylabel('y position (mm)')
ax.set_title('Zoomed Sample Position')
ax.legend()
ax.grid(True)
plt.tight_layout()
plt.show()
