function x_dot = ekf_state_transition(x, u, I_sat, I_sat_inv)
%   Non-linear state dynamics for EKF prediction step.
%   Calculates dx/dt = f(x,u)
%   x: Current state vector [q(4x1); omega(3x1); h_rw(3x1)] (10x1)
%   u: Control torque input [tau_cx; tau_cy; tau_cz] (3x1)

    %% Unpack State Vector
    q = x(1:4);         % Quaternion [q0; q1; q2; q3]
    omega = x(5:7);     % Angular Velocity [rad/s]
    h_rw = x(8:10);     % Wheel Momentum [Nms]

    %% Quaternion Kinematics: q_dot = 1/2 * Omega(omega) * q
    % Omega matrix
    Omega_mat = [0,        -omega(1), -omega(2), -omega(3);
                 omega(1),  0,         omega(3), -omega(2);
                 omega(2), -omega(3),  0,         omega(1);
                 omega(3),  omega(2), -omega(1),  0       ];
             
    q_dot = 0.5 * Omega_mat * q;

    %% Angular Velocity Dynamics (Euler's Equation):
    % omega_dot = I_sat_inv * (u - omega x H_total)
    % System angular momentum
    H_total = I_sat * omega + h_rw;
    % Gyroscopic coupling term
    coupling = cross(omega, H_total);
    % Angular acceleration
    omega_dot = I_sat_inv * (u - coupling); 

    %% Reaction Wheel Momentum Dynamics
    % h_rw_dot = -tau_wheel_on_body = -u
    h_rw_dot = -u;

    %% Pack Derivative Vector
    x_dot = [q_dot; omega_dot; h_rw_dot];
end