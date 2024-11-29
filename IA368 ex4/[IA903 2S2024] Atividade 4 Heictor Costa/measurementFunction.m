% function [h, H_x] = measurementFunction(x, m)
% % [h, H_x] = measurementFunction(x, m) returns the predicted measurement
% % given a state x and a single map entry m. H_x denotes the Jacobian of the
% % measurement function with respect to the state evaluated at the state
% % provided.
% % Map entry and state are defined according to "Introduction to Autonomous Mobile Robots" pp. 337
% 
% %STARTRM
% h = #;
% 
% H_x = #;
% 
% %ENDRM
% 
% [h(1), h(2), isRNegated] = normalizeLineParameters(h(1), h(2));
% 
% if isRNegated 
%     H_x(2, :) = - H_x(2, :);
% end
% 

function [h, H_x] = measurementFunction(x, m)
    % [h, H_x] = measurementFunction(x, m)
    % x: state vector [x, y, theta]
    % m: map entry [a, r]

    % Extract variables
    a = m(1); % angle of the line
    r = m(2); % distance of the line
    theta = x(3); % robot orientation
    x_pos = x(1); % robot x-position
    y_pos = x(2); % robot y-position

    % Predicted measurement
    h = [
        a - theta;
        r - (x_pos * cos(a) + y_pos * sin(a))
    ];

    % Jacobian with respect to the state
    H_x = [
        0, 0, -1;
        -cos(a), -sin(a), 0
    ];

    % Normalize the line parameters
    [h(1), h(2), isRNegated] = normalizeLineParameters(h(1), h(2));

    % Adjust Jacobian if r is negated
    if isRNegated
        H_x(2, :) = -H_x(2, :);
    end
end

