import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# Specify your serial port and baud rate
# You can find the correct port by checking your Arduino IDE or device manager (e.g., COM3, /dev/ttyUSB0)
serial_port = "COM10"  # Modify this to match your Arduino's serial port
baud_rate = 9600  # This should match the baud rate in your Arduino code

# Open the serial connection
ser = serial.Serial(serial_port, baud_rate)

# File to store the data

filename = r"D:\lduan\Documents\Individual Project\Test Data sets\sub8-test6.dat"


# Global variables for data storage
time_values = []

angle_values = []
posture_values = []
start_time = time.time()

print("Reading from Serial Port... Press CTRL+C to stop.")

try:
    while True:
        # Read a line from the serial monitor
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            print(f"Received data: {data}")
            
            # Skip non-numeric lines like "Connected to central"
            if "," not in data:
                continue
            
            try:
                # Parse the data (timestamp, tilt angle, posture status)
                parts = data.split(",")
                timestamp = float(parts[0])  # timestamp (millis)
                tilt_angle = float(parts[1])  # tilt angle
                posture_status = int(parts[2])  # posture status

                # Store the data
                time_values.append(timestamp / 1000)  # Convert to seconds
                angle_values.append(tilt_angle)
                posture_values.append(posture_status)

                # Write data to file
                with open(filename, "a") as file:
                    file.write(f"{timestamp:.2f} {tilt_angle:.2f} {posture_status}\n")
                    file.flush()  # Force write to disk
                    print("Data successfully written!")

            except ValueError:
                # Handle cases where the line is not in the expected format (e.g., a non-numeric value)
                print(f"Skipping invalid data line: {data}")
        
       
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nData collection stopped.")

finally:
    # Close the serial connection
    ser.close()

# Check if the data file has content
if len(time_values) > 0:
    # Plot Angular Position vs Time
    plt.figure(figsize=(10, 5))
    plt.subplot(2, 1, 1)
    plt.plot(time_values, angle_values, label="Tilt Angle (°)", color="blue")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (°)")
    plt.title("Head Tilt Over Time")
    plt.legend()
    plt.grid()

    # Plot Posture Detection vs Time
    plt.subplot(2, 1, 2)
    plt.plot(time_values, posture_values, label="Posture (0 = Good, 1 = Bad)", color="red", linestyle='dashed')
    plt.xlabel("Time (s)")
    plt.ylabel("Posture Status")
    plt.title("Posture Detection Over Time")
    plt.ylim(-0.5, 1.5)  # Keep y-axis readable
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show()

else:
    print("Error: No data collected.")
