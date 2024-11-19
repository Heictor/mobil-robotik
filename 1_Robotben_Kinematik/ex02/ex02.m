clc;

syms alpha beta gamma real

lb = 1; l1 = 1; l2 = 1; l3 = 1;

% Rotational matrices calculated in previous problem set
R_B1 = [1,0,0;0,cos(alpha),-sin(alpha);0,sin(alpha),cos(alpha)];
R_12 = [cos(beta),0,sin(beta);0,1,0;-sin(beta),0,cos(beta)];
R_23 = [cos(gamma),0,sin(gamma);0,1,0;-sin(gamma),0,cos(gamma)];

% Define the 3x1 relative position vectors for link lengths l_i = 1
r_B1_inB = [0; 0; lb];   % Position of link 1 in base frame B
r_12_in1 = [0; 0; l1];   % Position of link 2 in frame 1
r_23_in2 = [0; 0; l2];   % Position of link 3 in frame 2
r_3F_in3 = [0; 0; l3];   % Position of the foot point in frame 3

% Homogeneous transformation matrices
H_B1 = [R_B1, r_B1_inB; 0 0 0 1];  % Homogeneous transformation from frame B to frame 1
H_12 = [R_12, r_12_in1; 0 0 0 1];  % Homogeneous transformation from frame 1 to frame 2
H_23 = [R_23, r_23_in2; 0 0 0 1];  % Homogeneous transformation from frame 2 to frame 3

% Create the cumulative transformation matrix from frame B to frame 3
H_B2 = H_B1 * H_12;  % Cumulative transformation from frame B to frame 2
H_B3 = H_B2 * H_23;  % Cumulative transformation from frame B to frame 3

% Find the foot point position vector in the base frame
r_BF_inB = [...
    -sin(beta + gamma) - sin(beta);...
    sin(alpha) * (cos(beta + gamma) + cos(beta) + 1) + 1;...
    -cos(alpha) * (cos(beta + gamma) + cos(beta) + 1)];

% Validate the result using the provided valid.m script
valid
