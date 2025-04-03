import serial
import csv
from datetime import datetime

# === CONFIG ===
PORT = 'COM3'           # Change this to your ClearCore COM port (e.g., '/dev/ttyUSB0' on Linux/Mac)
BAUD_RATE = 9600
CSV_FILENAME = 'leveling_data_log.csv'

# === SETUP SERIAL AND CSV ===
ser = serial.Serial(PORT, BAUD_RATE, timeout=1)

with open(CSV_FILENAME, mode='w', newline='') as csvfile:
    writer = csv.writer(csvfile)

    # CSV header
    writer.writerow([
        "PC_Timestamp",      # Local PC datetime
        "Device_Time_ms",    # System time from ClearCore
        "LevelX",
        "LevelY",
        "inputVoltageX",
        "inputVoltageY",
        "inputVoltageSUM"
    ])

    print(f"Logging data from {PORT} to '{CSV_FILENAME}'... Press Ctrl+C to stop.\n")

    try:
        while True:
            line = ser.readline().decode('utf-8').strip()

            if not line or line.startswith("Time_ms"):  # skip header or empty lines
                continue

            try:
                parts = line.split(',')
                if len(parts) != 6:
                    print(f"⚠️ Invalid line: {line}")
                    continue

                # Parse and log
                now = datetime.now().isoformat()
                row = [now] + parts
                writer.writerow(row)
                print(', '.join(row))

            except Exception as e:
                print(f"❌ Error processing line: {line}\n{e}")

    except KeyboardInterrupt:
        print("\n✅ Logging stopped by user.")

    finally:
        ser.close()

