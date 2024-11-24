function [x_posteriori, P_posteriori] = incrementalLocalization(x, P, u, S, M, params, k, g, l)
    % Extract lines from laser scans
    C_TR = diag([repmat(0.1^2, 1, size(S, 2)), repmat(0.1^2, 1, size(S, 2))]);
    [z, R, ~] = extractLinesPolar(S(1, :), S(2, :), C_TR, params);
    
    % Plot measurements and prior
    z_prior = measurementFunction(x, M);
    figure(2), cla, hold on;
    plot(z(1, :), z(2, :), 'bo');
    plot(z_prior(1, :), z_prior(2, :), 'rx');
    xlabel('angle [rad]'); ylabel('distance [m]');
    legend('measurement', 'prior');
    drawnow;
    
    % EKF update
    [x_posteriori, P_posteriori] = filterStep(x, P, u, z, R, M, g, l);
end
