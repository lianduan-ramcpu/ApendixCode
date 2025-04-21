import pandas as pd
import numpy as np

# New data provided by user
data_new = {
     #  Task 1
    'Task 1 Mean': [3.58, 2.90, 1.90, 3.83, 4.01, 1.59, 3.53, 2.38],
    'Task 1 Max': [13.70, 5.90, 3.10, 7.60, 7.60, 7.00, 7.30, 4.90],
    'Task 1 Warning': [False] * 8,

    # Task 2
    'Task 2 Mean': [33.17, 31.42, 21.90, 32.10, 30.76, 27.40, 14.05, 26.48],
    'Task 2 Max': [37.70, 48.10, 31.50, 43.80, 36.60, 36.70, 20.00, 40.90],
    'Task 2 Warning': [True, True, False, True, True, True, False, True],

    # Task 3
    'Task 3 Mean': [3.03, 1.93, 1.36, 2.92, 5.25, 3.07, 2.14, 1.64],
    'Task 3 Max': [7.20, 18.40, 9.70, 6.80, 9.20, 7.90, 5.60, 4.20],
    'Task 3 Warning': [False] * 8,

    # Task 4
    'Task 4 Mean': [30.96, 29.86, 23.71, 25.46, 35.47, 29.20, 16.01, 27.16],
    'Task 4 Max': [35.80, 37.40, 30.80, 30.60, 40.00, 37.60, 18.50, 33.20],
    'Task 4 Warning': [True, True, False, False, True, True, False, True],

    # Task 5
    'Task 5 Mean': [13.98, 12.91, 22.09, 17.66, 21.45, 13.15, 11.98, 19.50],
    'Task 5 Max': [24.00, 21.20, 32.60, 21.60, 23.90, 16.70, 14.30, 26.20],
    'Task 5 Warning': [False, False, True, False, False, False, False, False],

    # Task 6 
    'Task 6 Max': [34.10, 23.80, 20.80, 28.20, 36.80, 35.60, 14.40, 29.10],
    'Task 6 Warning': [False, False, False, False, True, True, False, False],
}

# Convert to DataFrame for easy computation
df_new = pd.DataFrame(data_new)

# Function to compute basic statistics for a column
def compute_statistics(data):
    # Handle boolean columns separately
    if data.dtype == 'bool':
        return {
            "Count of True": np.sum(data),
            "Count of False": np.sum(~data),
        }
    else:
        return {
            "Mean": np.mean(data),
            "Max": np.max(data),
            "Min": np.min(data),
            "Standard Deviation": np.std(data, ddof=1),  
            "Range": np.max(data) - np.min(data),
            "IQR": np.percentile(data, 75) - np.percentile(data, 25)
        }

# Calculate statistics for the new dataset
statistics_new = {col: compute_statistics(df_new[col]) for col in df_new.columns}
stats_df_new = pd.DataFrame(statistics_new).T.round(2)
print(stats_df_new)