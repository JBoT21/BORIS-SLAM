"""Read collected measurements from serial monitor and determine variance of pitch and roll from accelerometer"""
import pandas as pd
import numpy as np

df = pd.read_csv('logs/angleMeasures2.csv', header=None, names=['Pitch', 'Roll'])
pitch_variance = np.var(df['Pitch'])
roll_variance = np.var(df['Roll'])

print(f"Pitch Variance: {pitch_variance:.4f}")
print(f"Roll Variance: {roll_variance:.4f}")