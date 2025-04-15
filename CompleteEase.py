import numpy as np
import matplotlib.pyplot as plt
import glob

# === CONFIG ===
cutoff = 25               # Time cutoff in minutes
num_points = 300          # Number of points for interpolation
data_dir = "EllipsometerLevelData"

# === LOAD BASELINES ===
baseline_motor = np.genfromtxt(f"{data_dir}/dynamic data.txt", delimiter='\t', skip_header=2, filling_values=np.nan)
baseline_adjustment = np.genfromtxt(f"{data_dir}/NoAdjustment.txt", delimiter='\t', skip_header=2, filling_values=np.nan)

# Extract and trim "No Motor" baseline
time_motor = baseline_motor[:, 0]
align_x_motor = baseline_motor[:, 1]
align_y_motor = baseline_motor[:, 2]
mask_motor = time_motor <= cutoff
time_motor = time_motor[mask_motor]
align_x_motor = align_x_motor[mask_motor]
align_y_motor = align_y_motor[mask_motor]

# Extract "No Adjustment" baseline
time_adjustment = baseline_adjustment[:, 0]
align_x_adjustment = baseline_adjustment[:, 1]
align_y_adjustment = baseline_adjustment[:, 2]

# === LOAD RUN DATA FILES ===
run_files = sorted(glob.glob(f"{data_dir}/Ellip_test*.txt"))
common_time = np.linspace(0, cutoff, num_points)
all_align_x, all_align_y = [], []

for file in run_files:
    data = np.genfromtxt(file, delimiter='\t', skip_header=2, filling_values=np.nan)
    time = data[:, 0]
    align_x = data[:, 1]
    align_y = data[:, 2]

    # Clip to cutoff
    mask = time <= cutoff
    time = time[mask]
    align_x = align_x[mask]
    align_y = align_y[mask]

    # Interpolate to common time base
    interp_x = np.interp(common_time, time, align_x)
    interp_y = np.interp(common_time, time, align_y)

    all_align_x.append(interp_x)
    all_align_y.append(interp_y)

# Convert to arrays for averaging
all_align_x = np.array(all_align_x)
all_align_y = np.array(all_align_y)
avg_align_x = np.mean(all_align_x, axis=0)
avg_align_y = np.mean(all_align_y, axis=0)

# === PLOT ALIGNX AND ALIGNY ===
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# --- ALIGN X ---
ax1.plot(time_motor, align_x_motor, label='No Motor', color='red')
ax1.plot(time_adjustment, align_x_adjustment, label='No Adjustment', color='blue')
for i, run in enumerate(all_align_x):
    ax1.plot(common_time, run, alpha=0.4, label=f'Run {i+1}')
ax1.plot(common_time, avg_align_x, color='black', linewidth=2, label='Average')
ax1.set_ylabel('AlignX')
ax1.set_title('AlignX vs Time')
ax1.legend(loc='upper right')
ax1.grid(True)

# --- ALIGN Y ---
ax2.plot(time_motor, align_y_motor, label='No Motor', color='red')
ax2.plot(time_adjustment, align_y_adjustment, label='No Adjustment', color='blue')
for i, run in enumerate(all_align_y):
    ax2.plot(common_time, run, alpha=0.4, label=f'Run {i+1}')
ax2.plot(common_time, avg_align_y, color='black', linewidth=2, label='Average')
ax2.set_xlabel('Time (min.)')
ax2.set_ylabel('AlignY')
ax2.set_title('AlignY vs Time')
ax2.legend(loc='upper right')
ax2.grid(True)

plt.tight_layout()
plt.show()
