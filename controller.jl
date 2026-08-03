# ---- REEL-OUT FLIGHT CONTROLLER ----

# 7 STATES
#  i_x_P_error       - Integrated x_P tracking error (m·s)
#  i_y_P_error       - Integrated y_P tracking error (m·s)
#  i_radius_P_error  - Integrated radius tracking error (m·s)
#  i_phi_R_error     - Integrated phi_R tracking error (deg·s)
#  i_alpha_error     - Integrated alpha tracking error (deg·s)
#  i_beta_error      - Integrated beta tracking error (deg·s)
#  filtered_radius_P - Low-pass filtered production-plane radius (m)

# INPUTS
#  phi, theta, psi - Earth-axis Euler angles (deg)
#  alpha, beta     - Angle of attack and sideslip (deg)
#  X, Y, Z         - Kite position in Earth axis coordinates (m)

# OUTPUTS
#  delta_a_SP - Demanded aileron deflection (deg)
#  delta_e_SP - Demanded elevator deflection (deg)
#  delta_r_SP - Demanded rudder deflection (deg)
#  *_dot      - Time derivatives of the 7 states

function controller(
    alpha, beta, phi, theta, psi, X, Y, Z,
    i_x_P_error, i_y_P_error, i_radius_P_error,
    i_phi_R_error, i_alpha_error, i_beta_error, 
    filtered_radius_P
)

    # Set points
    P_X = 200; P_Y = 0; P_Z = -150  # Production cylinder origin in Earth axis coordinates (m)
    lambda_P = 13; zeta_P = 0       # Production cylinder orientation (deg)
    radius_SP = 75                  # Radius set point (m)
    alpha_SP = 6; beta_SP = 0       # Angle of attack and sideslip set points (deg)

    # Gains. Variable names indicate the loop's outputs
    tau_radius_p = 10.0 # Low-pass filter cutoff frequency (rad/s)
    Kp_lambda_R  =  0.005; Ki_lambda_R =  0.008
    Kp_zeta_R    = -0.04;  Ki_zeta_R   = -0.006
    Kp_phi_R_SP  =  0.01;  Ki_phi_R_SP =  0.02
    Kp_dela      = -0.5;   Ki_dela     = -0.5;   K_theta_br_dela = -0.5
    Kp_dele      = -0.2;   Ki_dele     = -0.6
    Kp_delr      =  0.5;   Ki_delr     =  0.5
    Kg           = 0.0*9.81 # Optional alpha suppression gain

    # Outer loop
    production_origin_E = [P_X, P_Y, P_Z]
    kite_position_E = [X, Y, Z]
    q_EP = compute_production_plane_quaternion(lambda_P, zeta_P) # Eq. (A8)
    x_P, y_P, _ = compute_production_plane_coordinates(kite_position_E, q_EP, production_origin_E) # Eqs. (A9-A10)
    x_P_error = 0.0 - x_P
    y_P_error = 0.0 - y_P
    lambda_R = Kp_lambda_R * x_P_error + Ki_lambda_R * i_x_P_error
    zeta_R = Kp_zeta_R * y_P_error + Ki_zeta_R * i_y_P_error
    radius_P = sqrt(x_P^2 + y_P^2)
    radius_P_error = radius_SP - filtered_radius_P
    phi_R_SP = Kp_phi_R_SP * radius_P_error + Ki_phi_R_SP * i_radius_P_error

    # Inner loop
    q_EB = euler_to_quaternion(phi, theta, psi) # Eq. (A1)
    phi_R, theta_R, _ = compute_reference_frame_attitude(lambda_R, zeta_R, q_EB) # Eqs. (A2-A7)
    phi_R_error = phi_R_SP - phi_R
    delta_a_SP = Kp_dela * phi_R_error + Ki_dela * i_phi_R_error + K_theta_br_dela * theta_R
    alpha_error = alpha_SP - alpha
    delta_e_SP = Kp_dele * alpha_error + Ki_dele * i_alpha_error + Kg*sind(theta)
    beta_error = beta_SP - beta
    delta_r_SP = Kp_delr * beta_error + Ki_delr * i_beta_error

    # State derivatives
    i_x_P_error_dot       = x_P_error
    i_y_P_error_dot       = y_P_error
    i_radius_P_error_dot  = radius_P_error
    i_phi_R_error_dot     = phi_R_error
    i_alpha_error_dot     = alpha_error
    i_beta_error_dot      = beta_error
    filtered_radius_P_dot = -1/tau_radius_p * (filtered_radius_P - radius_P)

    return (
        delta_a_SP, delta_e_SP, delta_r_SP,
        i_x_P_error_dot, i_y_P_error_dot, i_radius_P_error_dot,
        i_phi_R_error_dot, i_alpha_error_dot, i_beta_error_dot,
        filtered_radius_P_dot
    )
end