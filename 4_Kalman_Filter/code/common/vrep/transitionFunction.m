function [f, F_x, F_u] = transitionFunction(x, u, l)
    % Extract state and inputs
    theta = x(3); 
    delta_sl = u(1); 
    delta_sr = u(2);

    % Compute motion parameters
    delta_s = (delta_sl + delta_sr) / 2; 
    delta_theta = (delta_sr - delta_sl) / l;

    % Predicted state
    f = [
        x(1) + delta_s * cos(theta + delta_theta / 2);
        x(2) + delta_s * sin(theta + delta_theta / 2);
        theta + delta_theta
    ];

    % Jacobian with respect to the state
    F_x = [
        1, 0, -delta_s * sin(theta + delta_theta / 2);
        0, 1,  delta_s * cos(theta + delta_theta / 2);
        0, 0, 1
    ];

    % Jacobian with respect to the inputs
    F_u = [
        0.5 * cos(theta + delta_theta / 2) - (delta_theta / (2 * l)) * sin(theta + delta_theta / 2), ...
        0.5 * cos(theta + delta_theta / 2) + (delta_theta / (2 * l)) * sin(theta + delta_theta / 2);
        0.5 * sin(theta + delta_theta / 2) + (delta_theta / (2 * l)) * cos(theta + delta_theta / 2), ...
        0.5 * sin(theta + delta_theta / 2) - (delta_theta / (2 * l)) * cos(theta + delta_theta / 2);
        -1 / l, 1 / l
    ];
end
