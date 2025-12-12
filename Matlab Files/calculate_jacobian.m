function A_num = calculate_jacobian(x_current, I_sat)
%   Calculates the Jacobian matrix A = df/dx at the current state x.
%   x_current: 10x1 state vector [q; omega; h_rw]

    %% Unpack current state estimate
    q0 = x_current(1); q1 = x_current(2); q2 = x_current(3); q3 = x_current(4);
    wx = x_current(5); wy = x_current(6); wz = x_current(7);
    h_rwx = x_current(8); h_rwy = x_current(9); h_rwz = x_current(10);
    
    Ixx = I_sat(1,1); Iyy = I_sat(2,2); Izz = I_sat(3,3);
    
    %% Matrix Structure based on Symbolic Derivation from symbolic_jacobian.mlx
    % The top-left 4x4 block depends on omega
    A_q_omega = 0.5 * [0, -wx, -wy, -wz;
                       wx, 0,   wz, -wy;
                       wy, -wz, 0,   wx;
                       wz, wy, -wx, 0];
    
    % The top-right 4x3 block depends on q
    A_q_q = 0.5 * [-q1, -q2, -q3;
                    q0, -q3,  q2;
                    q3,  q0, -q1;
                   -q2,  q1,  q0];
    
    % The middle block depends on omega, h_rw, and Inertias (Euler terms)
    A_omega_omega = zeros(3,3);
    A_omega_omega(1,2) = (Iyy*wz - Izz*wz - h_rwz)/Ixx;
    A_omega_omega(1,3) = (Iyy*wy - Izz*wy + h_rwy)/Ixx;
    A_omega_omega(2,1) = (Ixx*wz - Izz*wz + h_rwz)/Iyy;
    A_omega_omega(2,3) = (Ixx*wx - Izz*wx - h_rwx)/Iyy;
    A_omega_omega(3,1) = (Ixx*wy - Iyy*wy - h_rwy)/Izz;
    A_omega_omega(3,2) = (Ixx*wx - Iyy*wx + h_rwx)/Izz;
    
    % The middle-right block relates d(omega) to d(h_rw)
    A_omega_h = zeros(3,3);
    A_omega_h(1,2) = wz/Ixx; A_omega_h(1,3) = -wy/Ixx;
    A_omega_h(2,1) = -wz/Iyy; A_omega_h(2,3) = wx/Iyy;
    A_omega_h(3,1) = wy/Izz; A_omega_h(3,2) = -wx/Izz;
    
    % --- Assemble Full 10x10 Jacobian ---
    A_num = zeros(10,10);
    A_num(1:4, 1:4) = A_q_omega;
    A_num(1:4, 5:7) = A_q_q;
    A_num(5:7, 5:7) = A_omega_omega;
    A_num(5:7, 8:10) = A_omega_h;
    
    % Rows 8-10 are all zero because d(h_rw) = -tau (no state dependence)
end