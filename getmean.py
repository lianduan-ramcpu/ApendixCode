import numpy as np

# Replace this with your actual filename
filename = r"D:\lduan\Documents\Individual Project\Test Data sets\sub8-test6.dat"

# Load the data
data = np.loadtxt(filename)

# Separate time and angle
time_ms = data[:, 0]
angles = data[:, 1]
BooleanVal = data[:, 2]

# Filter out first 10 seconds and angle == 0
valid_indices = (time_ms >= 20000) & (angles != 0)
filtered_angles = angles[valid_indices]

# Calculate statistics
mean_angle = np.mean(filtered_angles)
max_angle = np.max(filtered_angles)


# Display results
print(f"Mean angle: {mean_angle:.2f}")
print(f"Max angle: {max_angle:.2f}")
if (BooleanVal==1).any():{print(f"True")}
else:print(f"False")