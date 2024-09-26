function [phi_c,chi_int] = courseHold(chi,chi_c,Vg,chi_int,h)

% Add your controller gains and course hold PI controller here
g = 9.81;
zeta_chi = 0.6;
w_n_phi =  0.5987;
W_chi = 10; % Value between 5 and 10

w_n_chi = (1/W_chi)*w_n_phi;
k_p_chi = 2 * zeta_chi * w_n_chi * (Vg/g);
k_i_chi = (Vg/g) * w_n_chi^2;

error_chi = ssa(chi_c - chi);
phi_c = error_chi * k_p_chi + k_i_chi * chi_int;

% Euler's method: chi_int[k+1]
chi_int = chi_int + h * ssa(chi_c - chi);

end
