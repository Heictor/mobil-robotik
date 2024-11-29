function [h, H_x] = measurementFunction(x, m)
% [h, H_x] = measurementFunction(x, m)
% Computes the predicted measurement given a robot state x and a single map entry m.
% Also computes the Jacobian H_x of the measurement function with respect to the state.

    % Extract state and map parameters
    x_t = x(1); % x-coordinate of the robot
    y_t = x(2); % y-coordinate of the robot
    theta_t = x(3); % orientation of the robot

    x_m = m(1); % x-coordinate of the map entry
    y_m = m(2); % y-coordinate of the map entry

    % Compute the predicted measurement
    dx = x_m - x_t; % Difference in x
    dy = y_m - y_t; % Difference in y
    r = sqrt(dx^2 + dy^2); % Distance to the landmark
    alpha = atan2(dy, dx) - theta_t; % Bearing to the landmark relative to the robot's orientation

    h = [r; alpha]; % Predicted measurement (distance and angle)

    % Compute the Jacobian H_x
    H_x = [
        -dx / r, -dy / r,  0;
         dy / r^2, -dx / r^2, -1
    ];

    % Normalize the line parameters and adjust Jacobian
    [h(1), h(2), isRNegated] = normalizeLineParameters(h(1), h(2));
    if isRNegated
        H_x(2, :) = -H_x(2, :);
    end
end
