clc; 

syms alpha beta gamma real;
load ca

lb = 1; l1 = 1; l2 = 1; l3 = 1;

% Rotation matrices calculated in the previous problem set
R_B1 = [1, 0, 0; 0, cos(alpha), -sin(alpha); 0, sin(alpha), cos(alpha)];
R_12 = [cos(beta), 0, sin(beta); 0, 1, 0; -sin(beta), 0, cos(beta)];
R_23 = [cos(gamma), 0, sin(gamma); 0, 1, 0; -sin(gamma), 0, cos(gamma)];

% 3x1 relative position vectors for link lengths l_i=1
r_B1_inB = [0; 0; lb];
r_12_in1 = [0; 0; l1];
r_23_in2 = [0; 0; l2];
r_3F_in3 = [0; 0; l3];

% Homogeneous transformation matrices
H_B1 = [R_B1, r_B1_inB; 0, 0, 0, 1];
H_12 = [R_12, r_12_in1; 0, 0, 0, 1];
H_23 = [R_23, r_23_in2; 0, 0, 0, 1];

% Create the cumulative transformation matrix
H_B3 = simplify(H_B1 * H_12 * H_23);

% Find the foot point position vector
r_BF_inB = H_B3 * [r_3F_in3; 1];
r_BF_inB = simplify(r_BF_inB(1:3)); % Extract only the position part
r_BF_inBx1 = eval(subs(r_BF_inB, [alpha beta gamma], q1));

disp('Cumulative Transformation Matrix H_B3:');
r_BF_inBx2 = eval(subs(r_BF_inB, [alpha beta gamma], q2));
disp(H_B3);

disp('Foot point position vector r_BF_inB:');
r_BF_inBx3 = eval(subs(r_BF_inB, [alpha beta gamma], q3));
disp(r_BF_inB);
 
save ca
valid
