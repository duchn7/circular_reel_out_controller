# ---- HELPER FUNCTIONS ----

# Appendix A: Earth-to-reference-plane Euler angle transformation
function compute_reference_frame_attitude(lambda_R, zeta_R, q_EB)
    q_z = [cosd(zeta_R/2),        0,           0,                     sind(zeta_R/2)]
    q_y = [cosd((lambda_R+90)/2), 0,           sind((lambda_R+90)/2), 0             ]
    q_x = [cosd(180/2),           sind(180/2), 0,                     0             ]

    q_ER = T(q_z) * T(q_y) * q_x # Eq. (A2)
    q_BR = T(quaternion_inverse(q_EB)) * q_ER # Eq. (A3)
    R_BR = quaternion_to_rotation_matrix(q_BR) # Eq. (A6)

    # Eq. (A7)
    phi_R   =  atand(R_BR[2, 3], R_BR[3, 3])
    theta_R = -asind(R_BR[1, 3])
    psi_R   =  atand(R_BR[1, 2], R_BR[1, 1])
    return phi_R, theta_R, psi_R
end

# Appendix B: production-plane quaternion (Eq. (A8))
function compute_production_plane_quaternion(lambda_P, zeta_P)
    q_z  = [cosd(zeta_P/2),        0,           0,                     sind(zeta_P/2)]
    q_y  = [cosd((lambda_P+90)/2), 0,           sind((lambda_P+90)/2), 0             ]
    q_x  = [cosd(180/2),           sind(180/2), 0,                     0             ]
    q_EP = T(q_z) * T(q_y) * q_x
    return q_EP
end

# Appendix B: Earth-to-production-plane coordinates (Eqs. (A9–A10))
function compute_production_plane_coordinates(kite_position_E, q_EP, prod_origin)
    q_PE     = quaternion_inverse(q_EP)
    R_PE     = quaternion_to_rotation_matrix(q_PE)
    pos_in_P = R_PE * (kite_position_E - prod_origin)
    x_P      = pos_in_P[1]
    y_P      = pos_in_P[2]
    z_P      = pos_in_P[3]
    return x_P, y_P, z_P
end

# Quaternion primitives
# Convention: q_AB = quaternion rotating frame B to frame A, stored [w, x, y, z].
# T(q1)*q2 = q1 ⊗ q2 (Hamilton product, Egeland & Gravdahl 2002, p. 232).

function euler_to_quaternion(phi, theta, psi) # Eq. (A1)
    q_phi   = [cosd(phi   / 2), sind(phi   / 2), 0,               0              ]
    q_theta = [cosd(theta / 2), 0,               sind(theta / 2), 0              ]
    q_psi   = [cosd(psi   / 2), 0,               0,               sind(psi   / 2)]
    return T(q_psi) * T(q_theta) * q_phi
end

function S(w) # Eq. (A5)
    return [ 0.0   -w[3]   w[2];
             w[3]   0.0   -w[1];
            -w[2]   w[1]   0.0 ]
end

function T(q)
    η, e1, e2, e3 = q[1], q[2], q[3], q[4]
    return [ η   -e1  -e2  -e3;
             e1   η   -e3   e2;
             e2   e3   η   -e1;
             e3  -e2   e1   η  ]
end

function quaternion_inverse(q_AB)
    return [q_AB[1], -q_AB[2], -q_AB[3], -q_AB[4]]
end

# Eq. (A4): returns R_AB
# Maps vectors from frame B to frame A (Egeland & Gravdahl Eq. (6.215)).
function quaternion_to_rotation_matrix(q_AB)
    η = q_AB[1]
    ε = q_AB[2:4]
    return I(3) + 2η*S(ε) + 2*S(ε)*S(ε)
end