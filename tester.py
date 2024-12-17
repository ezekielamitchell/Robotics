import numpy as np
import matplotlib.pyplot as plt

# Define the slope function
def f(t, P):
    return 0.06 * P * (1 - P / 2000)

# Generate grid points
t = np.linspace(0, 100, 20)  # Time from 0 to 100
P = np.linspace(0, 2500, 20)  # Population from 0 to 2500

T, P_grid = np.meshgrid(t, P)  # Create meshgrid
dP = f(T, P_grid)  # Calculate slopes

# Normalize the slopes for plotting
dt = np.ones_like(dP)
dP_norm = dP / np.sqrt(dt**2 + dP**2)

# Plot the direction field
plt.figure(figsize=(10, 6))
plt.quiver(T, P_grid, dt, dP_norm, angles="xy", scale_units="xy", scale=0.1, color="blue")
plt.xlabel("Time (t)")
plt.ylabel("Population (P)")
plt.title("Direction Field for Logistic Growth")
plt.grid()
plt.show()
