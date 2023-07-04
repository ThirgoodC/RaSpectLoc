import numpy as np
import matplotlib.pyplot as plt

# Data for different settings
x1 = [150]
y1 = [0.18542]

x2 = [230]
y2 = [0.15567]

x3 = [1570]
y3 = [0.17490]

x4 = [110]
y4 = [0.15966]

x5 = [90]
y5 = [0.18568]

x6 = [24]
y6 = [0.24568]

markers = [".", ",", "o", "v", "^", "<", ">"]

# Plot data for different settings on the same axes
plt.scatter(x1, y1, label='SLK', c="red", marker=markers[1], s=100)
plt.scatter(x2, y2, label='Mod.L2', c="green", marker=markers[2], s=100)
plt.scatter(x3, y3, label='Wasser.', c="blue", marker=markers[3], s=100)
plt.scatter(x4, y4, label='KL.D', c="magenta", marker=markers[4], s=100)
plt.scatter(x5, y5, label='SAM', c="black", marker=markers[5], s=100)
plt.scatter(x6, y6, label='AMCL', c="orange", marker=markers[6], s=100)  # Add x6 and y6 with unique marker and color

plt.xscale('log')

# Set plot title and axis labels
plt.title('Error vs. Time taken')
plt.xlabel('Time Taken (seconds)')
plt.ylabel('ATE:RMSE Error (m)')

# Add legend to plot
plt.legend()

# Add grid lines
plt.grid(True, which='both', axis='both', linestyle='-', linewidth=0.5)

# Display plot
plt.show()

