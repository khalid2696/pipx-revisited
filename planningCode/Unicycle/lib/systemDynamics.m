% Self-contained unicycle dynamics function handle (imported from funnel computation code)
function f = systemDynamics(x, u)
    
    % x = [p_x; p_y; theta]
    % u = [v; omega]
    
    f = [u(1) * cos(x(3)); ...  % x_dot
         u(1) * sin(x(3)); ...  % y_dot
         u(2)];                % theta_dot
end