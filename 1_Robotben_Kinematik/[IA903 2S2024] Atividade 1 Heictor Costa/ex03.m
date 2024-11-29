clc; 

syms alpha beta gamma real

q = [alpha; beta; gamma];

% Foot point position vector based on the provided formula
r_BF_inB = [...
    -sin(beta + gamma) - sin(beta);...
    sin(alpha)*(cos(beta + gamma) + cos(beta) + 1) + 1;...
    -cos(alpha)*(cos(beta + gamma) + cos(beta) + 1)];

% Determine the foot point Jacobian J_BF_inB = d(r_BF_inB)/dq
J_BF_inB = jacobian(r_BF_inB, q);

% Generalized velocity dq needed to lift the foot in vertical direction
% Given v = [0; 0; -1m/s] and q = [0; 60°; -120°]
v = [0; 0; -1];
qi = [0; 60*(pi/180); -120*(pi/180)];

% Determine the numerical value of the foot point Jacobian for the initial joint angles qi
JBF = double(subs(J_BF_inB, [alpha, beta, gamma], qi'));

% Determine the numerical value for dq: v = JBF * dq => dq = inv(JBF) * v
dq = JBF \ v;

valid
