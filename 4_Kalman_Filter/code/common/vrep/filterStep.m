function [x_posteriori, P_posteriori] = filterStep(x, P, u, Z, R, M, k, g, l)
    % State prediction
    Q = k * abs(u(1:2));
    [x_priori, F_x, F_u] = transitionFunction(x, u, l);
    P_priori = F_x * P * F_x' + F_u * Q * F_u';
    
    % Skip update if no measurements
    if isempty(Z)
        x_posteriori = x_priori;
        P_posteriori = P_priori;
        return;
    end
    
    % Measurement update
    [v, H, R] = associateMeasurements(x_priori, P_priori, Z, R, M, g);
    y = reshape(v, [], 1);
    H = reshape(permute(H, [1, 3, 2]), [], size(P, 1));
    R = blockDiagonal(R);
    
    % Innovation covariance
    S = H * P_priori * H' + R;
    K = P_priori * H' / S;
    
    % Posterior state and covariance
    x_posteriori = x_priori + K * y;
    P_posteriori = (eye(size(P)) - K * H) * P_priori;
end
