import pandas as pd
import numpy as np

df = pd.read_csv('logs/gyro_bias3.csv', header=None, names=['pitch_bias', 'pitch', 'roll_bias', 'roll'])

pitch_bias_trans_mean = np.mean(df['pitch_bias'][:10])
roll_bias_trans_mean = np.mean(df['roll_bias'][:10])

pitch_bias_mean = np.mean(df['pitch_bias'][100:])
pitch_bias_std = np.std(df['pitch_bias'][100:])

roll_bias_mean = np.mean(df['roll_bias'][100:])
roll_bias_std = np.std(df['roll_bias'][100:])

print(f"Initial Pitch Bias (first 10 samples): {pitch_bias_trans_mean:.5f}")
print(f"Initial Roll Bias (first 10 samples): {roll_bias_trans_mean:.5f}")
print(f"Pitch Bias: Mean = {pitch_bias_mean:.5f}, Std Dev = {pitch_bias_std:.5f}")
print(f"Roll Bias: Mean = {roll_bias_mean:.5f}, Std Dev = {roll_bias_std:.5f}")