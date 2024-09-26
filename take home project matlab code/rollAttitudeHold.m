function [delta_a_c,phi_int] = rollAttitudeHold(a_phi1,a_phi2,p,phi,phi_c,phi_int,h)

% Add your controller gains and PID roll attitude controller here

zeta_phi = 0.7;
delta_a_max = deg2rad(21);
error_phi_max = deg2rad(15);

% Natural frequency
w_n_phi = sqrt( abs(a_phi2) * (delta_a_max/error_phi_max));

k_p_phi = (delta_a_max / error_phi_max);
k_d_phi = (2 * zeta_phi * w_n_phi - a_phi1) / a_phi2;
k_i_phi = (k_p_phi * w_n_phi) / 10;

% Error
error_phi = ssa(phi_c-phi);

%delta_a_marked = k_p_phi*error_phi; % Without integral effect
delta_a_marked = k_p_phi*error_phi + k_i_phi * phi_int; % With integral effect


delta_a_c = -delta_a_marked - k_d_phi*p; 

% Euler's method: phi_int[k+1]
phi_int = phi_int + h * ssa(phi_c - phi);

end
