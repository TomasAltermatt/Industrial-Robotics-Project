function I_tot = inertia_matrix(Im, Ip)
% Im: Main moments of inertia [Ixx, Iyy, Izz]
% Ip: Products of inertia     [Ixy, Ixz, Iyz]
I_tot = [Im(1), Im(1), Ip(2); 
         Ip(1), Im(2), Ip(3); 
         Ip(2), Ip(3), Im(3)];
end
