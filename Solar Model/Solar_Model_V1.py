from math import sin, cos, acos, pi
import numpy as np
from scipy.optimize import minimize
from mpl_toolkits.mplot3d import Axes3D  # Use the correct import
import matplotlib.pyplot as plt

"""Parameters:"""
LT = 15
del_T_utc = 9.5
d = 231
La = -23.68
Lo = 133.87
G = 1360
LT_start = 8
LT_stop = 17

"""Calculating other Parameters:"""
B = 360 * (d - 81) / 365
LSTM = 15 * del_T_utc
EoT = 9.87 * sin(pi/180 * 2*B) - 7.53 * cos(pi/180 * B) - 1.5 * sin(pi/180 * B)
TC = 4 * (Lo - LSTM) + EoT
LST = LT + TC/60
HRA = 15 * (LST - 12)
Dec = -23.45 * cos(2*pi * (d + 10) / 365)
HRA_start = 15 * (LT_start + TC/60 - 12)
HRA_stop = 15 * (LT_stop + TC/60 - 12)

"""Calculating Power:"""

def Power(A, beta, gamma):
    Dec_r, beta_r, gamma_r, La_r, HRA_r = map(np.radians, (Dec, beta, gamma, La, HRA))

    theta = np.arccos(np.sin(Dec_r)*np.sin(La_r)*np.cos(beta_r) - np.sin(Dec_r)*np.cos(La_r)*np.sin(beta_r)*np.cos(gamma_r) +
                      np.cos(Dec_r)*np.cos(La_r)*np.cos(beta_r)*np.cos(HRA_r) + np.cos(Dec_r)*np.sin(La_r)*np.sin(beta_r)*np.cos(gamma_r)*np.cos(HRA_r) +
                      np.cos(Dec_r)*np.sin(beta_r)*np.sin(gamma_r)*np.sin(HRA_r))
    power = G * A * np.cos(theta)
    return power

# Returns energy in Joules
def Energy(A, beta, gamma):
  Dec_r, beta_r, gamma_r, La_r, HRA_start_r, HRA_stop_r = map(np.radians, (Dec, beta, gamma, La, HRA_start, HRA_stop))

  energy = 43200 * G / pi * (np.cos(La_r)*np.cos(Dec_r)*(np.sin(HRA_stop_r) - np.sin(HRA_start_r)) +
                             (HRA_stop_r - HRA_start_r)/180*np.sin(La_r)*np.sin(Dec_r))
  return energy * A

print(Power(6, 0, 0))

A = 6
def neg_Power(vars, A):
    x, y = vars
    return -Power(A, x, y)

initial_guess = [0, 0]
result = minimize(lambda vars: neg_Power(vars, A), initial_guess)

max_x, max_y = result.x
max_val = Power(A, max_x, max_y)
print(max_x, max_y, max_val)


# 3-D projection
ax = plt.axes(projection='3d')

# Defining all 3 axes
beta = np.linspace(-90, 90, 1800)
gamma = np.linspace(-90, 90, 1800)
Beta, Gamma = np.meshgrid(beta, gamma)

# Calculate power over the meshgrid
power_mesh = Power(6, Beta, Gamma)

# Plotting
ax.plot_surface(Beta, Gamma, power_mesh, cmap='viridis')
ax.set_title('Power received at different Panel angles')
ax.set_xlabel('Beta (deg)')
ax.set_ylabel('Gamma (deg)')
ax.set_zlabel('Power (W)')
plt.show()


print(Energy(6, 0, 0))






##### Testing stuff #####
# ene = []
# pow = []
# for i in range(0, 100):
#   beta, gamma = 0, 0
#   HRA_temp = (HRA_stop - HRA_start)*i/100 + HRA_start
#   Dec_r, beta_r, gamma_r, La_r, HRA_start_r, HRA_temp_r = map(np.radians, (Dec, beta, gamma, La, HRA_start, HRA_temp))

#   buf1 = 43200 * G / pi * (np.cos(La_r)*np.cos(Dec_r)*(np.sin(HRA_temp_r) - np.sin(HRA_start_r)) +
#                              (HRA_temp_r - HRA_start_r)/180*np.sin(La_r)*np.sin(Dec_r))
#   ene.append(buf1)

#   theta = np.arccos(np.sin(Dec_r)*np.sin(La_r)*np.cos(beta_r) - np.sin(Dec_r)*np.cos(La_r)*np.sin(beta_r)*np.cos(gamma_r) +
#                       np.cos(Dec_r)*np.cos(La_r)*np.cos(beta_r)*np.cos(HRA_temp_r) + np.cos(Dec_r)*np.sin(La_r)*np.sin(beta_r)*np.cos(gamma_r)*np.cos(HRA_temp_r) +
#                       np.cos(Dec_r)*np.sin(beta_r)*np.sin(gamma_r)*np.sin(HRA_temp_r))
#   buf2 = G * A * np.cos(theta)
#   pow.append(buf2)

# plt.plot(ene)
# plt.show()

# plt.plot(pow)
# plt.show()