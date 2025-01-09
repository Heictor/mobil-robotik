function [f, F_x, F_u] = transitionFunction(x, u, l)
% [f, F_x, F_u] = transitionFunction(x, u, l)
% Computes the predicted state at time t, the Jacobian w.r.t. the state, 
% and the Jacobian w.r.t. the input for a differential-drive robot.

    % Extract state and input variables
    x_t = x(1); % x-coordinate
    y_t = x(2); % y-coordinate
    theta_t = x(3); % orientation

    delta_sl = u(1); % Left wheel displacement
    delta_sr = u(2); % Right wheel displacement

    % Calculate the average displacement and change in orientation
    delta_s = (delta_sr + delta_sl) / 2; % Average displacement
    delta_theta = (delta_sr - delta_sl) / l; % Orientation change
    theta_mid = theta_t + delta_theta / 2; % Midpoint orientation

    % Calculate the predicted state (f)
    f = [
        x_t + delta_s * cos(theta_mid); % x-coordinate update
        y_t + delta_s * sin(theta_mid); % y-coordinate update
        theta_t + delta_theta           % orientation update
    ];

    % Calculate the Jacobian w.r.t. the state (F_x)
    F_x = [
        1, 0, -delta_s * sin(theta_mid); % Partial derivatives w.r.t. x_t, y_t, and theta_t
        0, 1,  delta_s * cos(theta_mid);
        0, 0, 1
    ];

    % Calculate the Jacobian w.r.t. the input (F_u)
    F_u = [
        0.5 * cos(theta_mid) + (delta_s / (2 * l)) * sin(theta_mid), ...
        0.5 * cos(theta_mid) - (delta_s / (2 * l)) * sin(theta_mid);
        0.5 * sin(theta_mid) - (delta_s / (2 * l)) * cos(theta_mid), ...
        0.5 * sin(theta_mid) + (delta_s / (2 * l)) * cos(theta_mid);
        -1 / l, 1 / l
    ];
end
