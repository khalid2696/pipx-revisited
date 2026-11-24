% Self-contained quadrotor dynamics function handle (imported from funnel computation code)
% along with system parameter values
function f = systemDynamics(x, u)
    
    quadParameters = systemParameters();
    %extracting quadrotor model parameters
    m = quadParameters.m; g = quadParameters.g;
    Jxx = quadParameters.J(1); Jyy = quadParameters.J(2); Jzz = quadParameters.J(3);

    %assigning state variables for ease of usage
    % State: x = [px; py; pz; vx; vy; vz; phi; theta; psi; p; q; r]

    %px = x(1); py = x(2); pz = x(3);
    vx = x(4); vy = x(5); vz = x(6);
    phi = x(7); theta = x(8); psi = x(9);
    p = x(10); q = x(11); r = x(12);

    %assigning input variables for ease of usage
    % Input: u = [T; Mx; My; Mz]
    T = u(1); Mp = u(2); Mq = u(3); Mr = u(4);

    % Trigonometric shortcuts
    c_phi = cos(phi); s_phi = sin(phi);
    c_theta = cos(theta); s_theta = sin(theta);
    c_psi = cos(psi); s_psi = sin(psi);
    t_theta = s_theta/c_theta;
    sec_theta = 1/c_theta;

    % Quadrotor dynamics
    % Assumptions: no aerodynamic drag and gyroscopic coupling due to rotor inertia)
    f = [
        % Position derivatives
        vx;
        vy;
        vz;

        % Velocity derivatives
        (T/m) * (c_phi * s_theta * c_psi + s_phi * s_psi);
        (T/m) * (c_phi * s_theta * s_psi - s_phi * c_psi);
        (T/m) * c_phi * c_theta - g;

        % Euler angle derivatives
        p + q * s_phi * t_theta + r * c_phi * t_theta;
        q * c_phi - r * s_phi;
        q * s_phi * sec_theta + r * c_phi * sec_theta;

        % Angular velocity derivatives
        (Mp + (Jyy - Jzz) * q * r) / Jxx;
        (Mq + (Jzz - Jxx) * p * r) / Jyy;
        (Mr + (Jxx - Jyy) * p * q) / Jzz
    ];
end

% Quadrotor Parameters (imported from funnel computation code) [!!keep both in sync!]
function params = systemParameters()
    params.m = 0.7;                  % mass (kg)
    params.g = 9.81;                 % gravity (m/s^2)
    params.J = [2e-3, 2e-3, 3.5e-3]; % moment of inertia (kg m^2)
end