include("controller.jl")
include("helper_functions.jl")

using LinearAlgebra

alpha = 6 # deg
beta = 0 # deg
phi = 0 # deg
theta = 90 # deg
psi = 180 # deg
X = 300 # m
Y = 0 # m
Z = -200 # m
i_x_P_error = 0 # m·s
i_y_P_error = 0 # m·s
i_radius_P_error = 0 # m·s
i_phi_R_error = 0 # deg·s
i_alpha_error = 0 # deg·s
i_beta_error = 0 # deg·s
filtered_radius_P = 50 # m

delta_a_SP, delta_e_SP, delta_r_SP,
i_x_P_error_dot, i_y_P_error_dot, i_radius_P_error_dot, i_phi_R_error_dot, i_alpha_error_dot, i_beta_error_dot,
filtered_radius_P_dot = 
    controller(alpha, beta, phi, theta, psi, X, Y, Z,
        i_x_P_error, i_y_P_error, i_radius_P_error,
        i_phi_R_error, i_alpha_error, i_beta_error, 
        filtered_radius_P)