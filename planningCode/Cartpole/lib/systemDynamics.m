% Self-contained cart-pole dynamics function handle (imported from funnel computation code)
% along with system parameter values
function f = systemDynamics(x, u)
    
    cartPoleParameters = systemParameters();

    %extract parameters
    M = cartPoleParameters.M; m = cartPoleParameters.m;
    L = cartPoleParameters.L; g = cartPoleParameters.g;

    % Extract states
    p_x = x(1);
    v_x = x(2);
    theta = x(3);
    omega = x(4);
    
    % Control input
    F = u;
    
    % Define trigonometric functions
    s_theta = sin(theta);
    c_theta = cos(theta);
    
    % Common denominator
    %denom = M + m*(1 - c_theta^2);
    denom = M + m*s_theta^2;
    
    % State derivatives
    f = [
        v_x;
        (F + m*L*omega^2*s_theta + m*g*s_theta*c_theta) / denom;
        omega;
        (-F*c_theta - m*L*omega^2*s_theta*c_theta - (M + m)*g*s_theta) / (L * denom)
    ];
end

% Quadrotor Parameters (imported from funnel computation code) [!!keep both in sync!]
function params = systemParameters()
    params.M = 1.0;    % cart mass (kg)
    params.m = 0.1;    % pendulum mass (kg)
    params.L = 0.5;    % pendulum length (m)
    params.g = 9.81;   % gravity (m/s^2)
end