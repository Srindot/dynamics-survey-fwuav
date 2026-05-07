import nbformat
from nbformat.v4 import new_markdown_cell, new_code_cell

# Load the notebook
with open('output/flight_analysis.ipynb', 'r') as f:
    nb = nbformat.read(f, as_version=4)

# Create markdown cell
md_content = """### 3. Wing Strip Physical Variables
Analyzing the instantaneous angles and forces on a mid-span wing strip (strip_l_5)."""
nb.cells.append(new_markdown_cell(md_content))

# Create code cell 1: Angles
code1 = """fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(df['time'], df['strip_l_5_alpha_geom'], label='Geom AOA')
ax.plot(df['time'], df['strip_l_5_epsilon_ce'], label='Twist (epsilon)')
ax.plot(df['time'], df['strip_l_5_alpha_e'], label='Effective AOA')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle (rad)')
ax.set_title('Mid-span Strip Angles over Time')
ax.legend()
ax.grid(True)
plt.savefig('../plot/strip_angles.png')
plt.show()"""
nb.cells.append(new_code_cell(code1))

# Create code cell 2: Forces
code2 = """fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(df['time'], df['strip_l_5_F_tran_x'], label='Translational Fx (Thrust)')
ax.plot(df['time'], df['strip_l_5_F_tran_z'], label='Translational Fz (Lift)')
ax.plot(df['time'], df['strip_l_5_F_rot_z'], label='Rotational Fz')
ax.plot(df['time'], df['strip_l_5_F_tot_x'], label='Total Fx', linestyle='--', linewidth=2)
ax.plot(df['time'], df['strip_l_5_F_tot_z'], label='Total Fz', linestyle='--', linewidth=2)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Force (N)')
ax.set_title('Mid-span Strip Forces over Time')
ax.legend()
ax.grid(True)
plt.savefig('../plot/strip_forces.png')
plt.show()"""
nb.cells.append(new_code_cell(code2))

# Save the notebook
with open('output/flight_analysis.ipynb', 'w') as f:
    nbformat.write(nb, f)
print("Notebook updated successfully.")
