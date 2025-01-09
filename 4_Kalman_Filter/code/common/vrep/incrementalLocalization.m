% function [x_posterori, P_posterori] = incrementalLocalization(x, P, u, S, M, params, k, g, l)
% % [x_posterori, P_posterori] = incrementalLocalization(x, P, u, S, R, M,
% % k, l, g) returns the a posterori estimate of the state and its covariance,
% % given the previous state estimate, control inputs, laser measurements and
% % the map
% 
% C_TR = diag([repmat(0.1^2, 1, size(S, 2)) repmat(0.1^2, 1, size(S, 2))]);
% [z, R, ans] = extractLinesPolar(S(1,:), S(2,:), C_TR, params);
% 
% 
% %STARTRM
% figure(2), cla, hold on;
% 
% %compute z_prior
% z_prior =#; %hint: several steps to get z_prior
% #
% #
% #
% 
% plot(z(1,:), z(2,:),'bo');
% plot(z_prior(1,:), z_prior(2,:),'rx');
% xlabel('angle [rad]'); ylabel('distance [m]')
% legend('measurement','prior')
% drawnow
% 
% % estimate robot pose
% [x_posterori, P_posterori] = #; %hint: you just coded this function
% 
% %ENDRM

function [x_posteriori, P_posteriori] = incrementalLocalization(x, P, u, S, M, params, k, g, l)
    % [x_posteriori, P_posteriori] = incrementalLocalization(x, P, u, S, M, params, k, g, l)
    % Computes the a posteriori estimate of the robot's state and covariance.

    % Define covariance matrix for laser measurements
    C_TR = diag([repmat(0.1^2, 1, size(S, 2)), repmat(0.1^2, 1, size(S, 2))]);

    % Extract lines from laser scan in polar coordinates
    [z, R, ~] = extractLinesPolar(S(1, :), S(2, :), C_TR, params);

    %STARTRM
    figure(2), cla, hold on;

    % Compute the predicted measurements (z_prior)
    z_prior = zeros(size(z)); % Initialize z_prior
    for i = 1:size(M, 2)
        % For each map line, predict the measurement based on current pose
        [z_prior(:, i), ~] = measurementFunction(x, M(:, i));
    end

    % Visualize measurements and predictions
    plot(z(1, :), z(2, :), 'bo'); % Measured lines (blue circles)
    plot(z_prior(1, :), z_prior(2, :), 'rx'); % Predicted lines (red crosses)
    xlabel('angle [rad]');
    ylabel('distance [m]');
    legend('Measurement', 'Prediction');
    drawnow;

    % Estimate robot pose using the filter step
    [x_posteriori, P_posteriori] = filterStep(x, P, u, z, R, M, k, g, l);

    %ENDRM
end
