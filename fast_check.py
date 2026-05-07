import csv
import math

pitch_vals = []
vx_vals = []

with open('plot/simulation_data.csv', 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        pitch_vals.append(float(row['pitch']))
        vx_vals.append(float(row['vel_x']))

min_pitch = min(pitch_vals)
max_pitch = max(pitch_vals)
final_pitch = pitch_vals[-1]

sign_changes = sum(1 for i in range(1, len(vx_vals)) if vx_vals[i]*vx_vals[i-1] < 0)

print(f"Pitch bounds (deg): {math.degrees(min_pitch):.1f} to {math.degrees(max_pitch):.1f}")
print(f"Final pitch (deg): {math.degrees(final_pitch):.1f}")
print(f"Num sign changes in vx: {sign_changes}")
