function [delta_r_c, beta_int] = sideslipHold(a_beta1,a_beta2,beta,beta_int,h)

% Add your controller gains and sideslip PI controller here
delta_r_max = deg2rad(30);
e_beta_max = deg2rad(20);
zeta_b = 0.8;
beta_c = 0;

k_p_beta = (delta_r_max / e_beta_max);
w_n_beta = (a_beta1 + a_beta2 * k_p_beta) / (2 * zeta_b);
k_i_beta = w_n_beta^2 / a_beta2;            
                            
e_beta = beta_c-beta;
delta_r_c = e_beta * k_p_beta + k_i_beta * beta_int;

% Euler's method: beta_int[k+1]
beta_int = beta_int + h * e_beta;

end





