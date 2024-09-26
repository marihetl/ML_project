function [alpha,beta,Va,Vg,Vw] = windTriangle(v_g,v_w)
% [alpha,beta,Va,Vg,Vw] = windTriangle(v_g,v_w)


% Relation between the velocities 
v_a = v_g - v_w;

u_r = v_a(1);
v_r = v_a(2);
w_r = v_a(3);

Va = norm(v_a);
Vg = norm(v_g);
Vw = norm(v_w);

alpha = rad2deg(atan(w_r/u_r));
beta = rad2deg(asin(v_r/Va)); 

end
