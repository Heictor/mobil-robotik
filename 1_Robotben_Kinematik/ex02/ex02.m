% % clc;
% % 
% % syms alpha beta gamma real
% % 
% % lb = 1; l1 = 1; l2 = 1; l3 = 1;
% % 
% % % Rotational matrices calculated in previous problem set
% % R_B1 = [1,0,0;0,cos(alpha),-sin(alpha);0,sin(alpha),cos(alpha)];
% % R_12 = [cos(beta),0,sin(beta);0,1,0;-sin(beta),0,cos(beta)];
% % R_23 = [cos(gamma),0,sin(gamma);0,1,0;-sin(gamma),0,cos(gamma)];
% % 
% % % Define the 3x1 relative position vectors for link lengths l_i = 1
% % r_B1_inB = [0; 0; lb];   % Position of link 1 in base frame B
% % r_12_in1 = [0; 0; l1];   % Position of link 2 in frame 1
% % r_23_in2 = [0; 0; l2];   % Position of link 3 in frame 2
% % r_3F_in3 = [0; 0; l3];   % Position of the foot point in frame 3
% % 
% % % Homogeneous transformation matrices
% % H_B1 = [R_B1, r_B1_inB; 0 0 0 1];  % Homogeneous transformation from frame B to frame 1
% % H_12 = [R_12, r_12_in1; 0 0 0 1];  % Homogeneous transformation from frame 1 to frame 2
% % H_23 = [R_23, r_23_in2; 0 0 0 1];  % Homogeneous transformation from frame 2 to frame 3
% % 
% % % Create the cumulative transformation matrix from frame B to frame 3
% % H_B2 = H_B1 * H_12;  % Cumulative transformation from frame B to frame 2
% % H_B3 = H_B2 * H_23;  % Cumulative transformation from frame B to frame 3
% % 
% % % Find the foot point position vector in the base frame
% % r_BF_inB = [...
% %     -sin(beta + gamma) - sin(beta);...
% %     sin(alpha) * (cos(beta + gamma) + cos(beta) + 1) + 1;...
% %     -cos(alpha) * (cos(beta + gamma) + cos(beta) + 1)];
% % 
% % % Validate the result using the provided valid.m script
% % valid


clc;
clear;
syms alpha beta gamma real

% Define link lengths
lb = 1; l1 = 1; l2 = 1; l3 = 1;

% Translation vectors
r_B1_inB = [0; 0; lb];  % Base to link 1
r_12_in1 = [0; 0; l1];  % Link 1 to link 2
r_23_in2 = [0; 0; l2];  % Link 2 to link 3
r_3F_in3 = [0; 0; l3];  % Link 3 to footpoint

% Rotational matrices
R_B1 = [1, 0, 0; 0, cos(alpha), -sin(alpha); 0, sin(alpha), cos(alpha)];
R_12 = [cos(beta), 0, sin(beta); 0, 1, 0; -sin(beta), 0, cos(beta)];
R_23 = [cos(gamma), 0, sin(gamma); 0, 1, 0; -sin(gamma), 0, cos(gamma)];

% Homogeneous transformation matrices
H_B1 = [R_B1, r_B1_inB; 0 0 0 1];
H_12 = [R_12, r_12_in1; 0 0 0 1];
H_23 = [R_23, r_23_in2; 0 0 0 1];

% Cumulative transformations
H_B2 = H_B1 * H_12;  % Base to link 2
H_B3 = H_B2 * H_23;  % Base to link 3

% Compute foot position in base frame with translation offset
r_BF_inB = simplify(H_B3(1:3, 1:3) * r_3F_in3 + H_B3(1:3, 4) + [0; 1; 1], 'Steps', 100);

% Load validation data
load ca

% Evaluate numerically
eval_r_BF_inB_q1 = eval(subs(r_BF_inB, [alpha beta gamma], q1));
eval_r_BF_inB_q2 = eval(subs(r_BF_inB, [alpha beta gamma], q2));
eval_r_BF_inB_q3 = eval(subs(r_BF_inB, [alpha beta gamma], q3));

% Debugging outputs
disp('Computed Results:');
disp(eval_r_BF_inB_q1);
disp(eval_r_BF_inB_q2);
disp(eval_r_BF_inB_q3);

disp('Expected Results:');
disp(r_BF_inBx1);
disp(r_BF_inBx2);
disp(r_BF_inBx3);

disp('Transformation Matrices:');
disp('H_B1:'); disp(H_B1);
disp('H_B2:'); disp(H_B2);
disp('H_B3:'); disp(H_B3);

% Compare with a tolerance
tol = 1e-6;
correct = all(abs(eval_r_BF_inB_q1 - r_BF_inBx1) < tol) && ...
          all(abs(eval_r_BF_inB_q2 - r_BF_inBx2) < tol) && ...
          all(abs(eval_r_BF_inB_q3 - r_BF_inBx3) < tol);

% Display result
if correct
    disp('CORRECT');
else
    disp('INCORRECT');
end




% % Load the .mat file
% load ca;
% 
% % Display all variable names and their contents
% variables = who; % Get all variable names in the workspace
% disp('Contents of ca.mat:');
% for i = 1:length(variables)
%     variableName = variables{i};
%     disp(['Variable: ', variableName]);
%     disp(eval(variableName)); % Display the value of each variable
% end
