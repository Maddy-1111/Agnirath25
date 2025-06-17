
battery_cap = 5.3 # kWh
battery_eff = 0.998
panel_area = 4 # m^2
panel_eff = 0.235
MPPT_eff = 0.98
motor_eff = 0.98
wheel_radius = 19/39.37 # m
mass = 280 # Kg
drag_coeff = 0.12
frontal_area = 1 # m^2
density_air = 1.225 # Kg/m^3
headwind = 1 # m/s
avg_velocity = 20 # m/s
friction_coeff = 0.0045


aero_drag_force = 1/2*1.225*((headwind+avg_velocity)**2)*drag_coeff*frontal_area
# print(aero_drag_force)
aero_energy_loss = aero_drag_force*avg_velocity*42/1000
print(aero_energy_loss)

friction_force = friction_coeff*mass*9.81
print(friction_force)


irradience = [[86, 301, 531, 717, 859, 936, 932, 855, 708, 515],
[164, 403, 637, 851, 986, 1046, 1029, 936, 776, 569],
[143, 397, 645, 845, 977, 1039, 1027, 940, 786, 576],
[244, 493, 723, 912, 1034, 1089, 1067, 982, 825, 617],
[309, 535, 727, 893, 993, 1026, 990, 889, 731, 530]]        # Unit - W/m^2

tot_energy = 0
for i in irradience:
    for j in i:
        tot_energy += j
tot_energy *= 0.22*4*3600       # Unit - J
tot_energy_kWh = tot_energy/(3600*1000)

# print(tot_energy_kWh)
