#!/usr/bin/env python3

import csv
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from scipy.interpolate import interp2d, interp1d
from scipy import sparse
from scipy.sparse.linalg import spsolve

from BaselineRemoval import BaselineRemoval

n_interpolated_points = 512

def add_shot_noise(spectrum, snr):
    """
    Adds shot noise to a Raman spectrum.
    
    Args:
        spectrum (numpy.ndarray): The Raman spectrum to add noise to.
        snr (float): The desired signal-to-noise ratio.
        
    Returns:
        numpy.ndarray: The noisy spectrum.
    """
    # Calculate the mean signal intensity and noise level
    signal_level = np.mean(spectrum)
    noise_level = signal_level / snr
    
    # Generate random noise samples
    noise = np.random.normal(scale=noise_level, size=len(spectrum))
    
    # Add the noise to the spectrum
    noisy_spectrum = spectrum + noise
    
    return noisy_spectrum

def baseline_als_optimized(y, lam, p, niter=10):
    L = len(y)
    D = sparse.diags([1,-2,1],[0,-1,-2], shape=(L,L-2))
    D = lam * D.dot(D.transpose()) # Precompute this term since it does not depend on `w`
    w = np.ones(L)
    W = sparse.spdiags(w, 0, L, L)
    for i in range(niter):
        W.setdiag(w) # Do not create a new matrix, just update diagonal values
        Z = W + D
        z = spsolve(Z, w*y)
        w = p * (y > z) + (1-p) * (y < z)
    return z

def baseline_als(y, lam, p, niter=10):
  L = len(y)
  D = sparse.diags([1,-2,1],[0,-1,-2], shape=(L,L-2))
  w = np.ones(L)
  for i in range(len(y)):
    W = sparse.spdiags(w, 0, L, L)
    Z = W + lam * D.dot(D.transpose())
    z = spsolve(Z, w*y)
    w = p * (y > z) + (1-p) * (y < z)
  return z

# files = ["1000056_2.csv",
#          "1000111_2.csv",
#          "1000025_2.csv",
#          "1000078_2.csv",
#          "1000061_2.csv",
#          "1000042_2.csv",
#          "1000103_2.csv",
#          "1000044_2.csv"
#          ]
# files = ["1000056_2.csv",
#          "1000162_2.csv",
#          "1000025_2.csv"
# ]

# files = ["../raman_scans_1/alizarin-Raman785nm.csv"]
files = ["1000150.csv"]

data = None
x_files = []
y_files = []
# x_files = np.array([], [],)
# y_files = np.array([])

for i in range(len(files)):
    with open(files[i],newline='') as csv_file:
        reader = csv.reader(csv_file, delimiter=' ')
        next(csv_file)  # discard header
        data = list(reader)

    x1 = [float(row[0]) for row in data]
    y1 = [float(row[1]) for row in data]
    
    x_files.append(x1)
    y_files.append(y1)
    
    print(x_files)



# for i in range(len(files)):
#     with open(files[i],newline=',') as csv_file:
#         reader = csv.reader(csv_file, delimiter=',')
#         next(csv_file)  # discard header
#         data = list(reader)

#     x1 = [float(row[0]) for row in data]
#     y1 = [float(row[1]) for row in data]

#     x_files.append(x1)
#     y_files.append(y1)

#     print(files[i])

# Actual group boundries
# startX0 = [x_files[0][0], x_files[1][0], x_files[2][0], x_files[3][0], x_files[4][0], x_files[5][0], x_files[6][0], x_files[7][0]]
# x1axis0th = max(startX0)
# endNx = [x_files[0][-1], x_files[1][-1], x_files[2][-1], x_files[3][-1], x_files[4][-1], x_files[5][-1], x_files[6][-1], x_files[7][-1]]
# x1axisNth = min(endNx)

# startX0 = [x_files[0][0], x_files[1][0], x_files[2][0]]
# x1axis0th = max(startX0)
# endNx = [x_files[0][-1], x_files[1][-1], x_files[2][-1]]
# x1axisNth = min(endNx)
# xnew = np.linspace(x1axis0th, x1axisNth, n_interpolated_points)

#TESTING baseline
# startX0 = [x_files[0][0]]
# x1axis0th = max(startX0)
# endNx = [x_files[0][-1]]
# x1axisNth = min(endNx)

startX0 = [x_files[0][0]]
# x1axis0th = max(startX0)
endNx = [x_files[0][-1]]
# x1axisNth = min(x_files[0][-1])
xnew = np.linspace(startX0, endNx, n_interpolated_points)
# #TESTING baseline fix
# plt.plot(baseline_als(x_files[0],y_files[0], 10000, 0.05))
# plt.show()

# baseObj=BaselineRemoval(y_files[0])
# Zhangfit_output=baseObj.ZhangFit()
# plt.plot(x_files[0],Zhangfit_output)
# plt.show()

#interpolation
axis_interp_y = []
for i in range(len(files)):
    # baseObj=BaselineRemoval(y_files[i])
    # Zhangfit_output=baseObj.ZhangFit()
    # axis_interp_y.append(interp1d(x_files[i], Zhangfit_output))
    axis_interp_y.append(interp1d(x_files[i], y_files[i]))


#normalisation
# norm_data = []
# for x in range(len(files)):
#     raw = []
#     for i in range(len(xnew)):
#         raw.append(axis_interp_y[x](xnew[i]))
#     norm_data.append([(float(j) * 7)/sum(raw) for j in raw])

# norm = [float(i)/sum(raw) for i in raw]

# for i in range(len(files)):
#     f = open(f"spec_{i}_inter_{n_interpolated_points}_x7_sum_norm_baserem.csv", "x")
#     for j in range(len(xnew)):
#         # intensity = axis_interp_y[i](xnew[j])
#         intensity = norm_data[i][j]
#         if intensity < 0:
#             intensity = -intensity
#         f.write(f"{xnew[j]},{intensity},")
#         f.write("\n")

#     f.close()
# raw = []
# for i in range(len(files)):
#     f = open(f"spec_{i}_inter_{n_interpolated_points}.csv", "x")

#     for j in range(len(xnew)):
#         # raw.append(axis_interp_y[i](xnew[j]))
#         intensity = axis_interp_y[i](xnew[j])
#         if intensity < 0:
#             intensity = 0.0
#         f.write(f"{xnew[j]},{intensity},")
#         f.write("\n")
        
#         # raw.append(intensity)

#     f.close()

# f = open(f"spec_2_inter_{n_interpolated_points}_2.csv", "x")

# for j in range(len(xnew)):
#     # raw.append(axis_interp_y[i](xnew[j]))
#     intensity = axis_interp_y[1](xnew[j])
#     if intensity < 0:
#         intensity = 0.0
#     f.write(f"{xnew[j]},{intensity},")
#     f.write("\n")
    
#     # raw.append(intensity)

# f.close()

raw = []    

for j in range(len(xnew)): 
    intensity = axis_interp_y[0](xnew[j])
    if intensity < 0:
        intensity = 0.0
    raw.append(intensity)
    
print(len(raw))

noisy_spectra = add_shot_noise(raw, 10)
# weights = np.arange(1, 513)

# plt.scatter(xnew, raw, c=weights, cmap='blues')
plt.plot(xnew, raw)
# plt.plot(xnew, raw, 'g',
#          xnew, noisy_spectra, 'b'
#          )
plt.show()
# plt.plot(xnew, axis_interp_y[0](xnew), 'b',
#          xnew, axis_interp_y[1](xnew), 'g',
#          xnew, axis_interp_y[2](xnew), 'y',
#          xnew, axis_interp_y[3](xnew), 'r',
#          xnew, axis_interp_y[4](xnew), 'm',
#          xnew, axis_interp_y[5](xnew), 'k',
#          xnew, axis_interp_y[6](xnew), 'c',
#          xnew, axis_interp_y[7](xnew), 'b',
#          )

# plt.plot(xnew, norm_data[0], 'b',
#          xnew, norm_data[1], 'g',
#          xnew, norm_data[2], 'y',
#          xnew, norm_data[3], 'r',
#          xnew, norm_data[4], 'm',
#          xnew, norm_data[5], 'k',
#          xnew, norm_data[6], 'c',
#          xnew, norm_data[7], 'b',
#          )
         
# plt.show()

# plt.plot(xnew, norm_data[0], 'g')
plt.show()

# y = np.arange(-5.01, 5.01, 0.25)
# f = interpolate.interp2d(xnew, x1, y1)