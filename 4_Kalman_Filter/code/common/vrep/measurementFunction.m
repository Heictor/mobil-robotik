function [h, H_x] = measurementFunction(x, m)
% [h, H_x] = measurementFunction(x, m) returns the predicted measurement
% given a state x and a single map entry m. H_x denotes the Jacobian of the
% measurement function with respect to the state evaluated at the state
% provided.
% Map entry and state are defined according to "Introduction to Autonomous Mobile Robots" pp. 337

%STARTRM
% Extract state and map entry
    x_r = x(1); y_r = x(2); theta_r = x(3);
    alpha_w = m(1); r_w = m(2);
    
    % Transform to body frame
    alpha_r = alpha_w - theta_r;
    r_r = r_w - (x_r * cos(alpha_w) + y_r * sin(alpha_w));
    h = [alpha_r; r_r];
    
    % Compute Jacobian
    H_x = [0, 0, -1;
           -cos(alpha_w), -sin(alpha_w), 0];
    
    % Normalize line parameters
    [h(1), h(2), isRNegated] = normalizeLineParameters(h(1), h(2));
    if isRNegated
        H_x(2, :) = -H_x(2, :);
    end
end

