% Make sure to have the simulation scene mooc_exercise.ttt running in V-REP!

% simulation setup, will add the matlab paths
connection = simulation_setup();

% the robot we want to interact with
robotNb = 0;

% open the connection
connection = simulation_openConnection(connection, robotNb);

% start simulation if not already started
simulation_start(connection);

vrep=connection.vrep;
% initialize connection
[err dt]=vrep.simxGetFloatingParameter(connection.clientID,vrep.sim_floatparam_simulation_time_step,vrep.simx_opmode_oneshot_wait);

% now enable stepped simulation mode:
simulation_setStepped(connection,true);

% given are the functions 
%   r_BF_inB(alpha,beta,gamma) and
%   J_BF_inB(alpha,beta,gamma) 
% for the foot positon respectively Jacobian

r_BF_inB = @(alpha,beta,gamma)[...
    -sin(beta + gamma) - sin(beta);...
  sin(alpha)*(cos(beta + gamma) + cos(beta) + 1) + 1;...
  -cos(alpha)*(cos(beta + gamma) + cos(beta) + 1)];
 
J_BF_inB = @(alpha,beta,gamma)[...
                                              0,             - cos(beta + gamma) - cos(beta),            -cos(beta + gamma);...
 cos(alpha)*(cos(beta + gamma) + cos(beta) + 1), -sin(alpha)*(sin(beta + gamma) + sin(beta)), -sin(beta + gamma)*sin(alpha);...
 sin(alpha)*(cos(beta + gamma) + cos(beta) + 1),  cos(alpha)*(sin(beta + gamma) + sin(beta)),  sin(beta + gamma)*cos(alpha)];
 
% write an algorithm for the inverse kinematics problem to
% find the generalized coordinates q that gives the endeffector position rGoal =
% [0.2,0.5,-2]' and store it in qGoal
q0 = pi/180*([0,-30,60])';
updatePos(vrep,connection.clientID,q0)
pause(0.5)

rGoal = [0.2,0.5,-2]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% enter here your algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set the initial guess for the joint angles (q)
q = q0;

% Set a convergence threshold and maximum number of iterations
threshold = 1e-5;
maxIterations = 100;
iteration = 0;
error = Inf;  % Initialize error to a large value

% Iteratively solve for qGoal using Newton-Raphson method
while norm(error) > threshold && iteration < maxIterations
    % Extract current joint angles (alpha, beta, gamma)
    alpha = q(1);
    beta = q(2);
    gamma = q(3);
    
    % Compute the current end-effector position
    rCurrent = r_BF_inB(alpha, beta, gamma);
    
    % Compute the error between the current and desired end-effector positions
    error = rGoal - rCurrent;
    
    % Compute the Jacobian at the current joint angles
    J = J_BF_inB(alpha, beta, gamma);
    
    % Compute the change in q using the inverse Jacobian
    delta_q = pinv(J) * error;  % Use pseudoinverse in case J is not square or singular
    
    % Update the joint angles
    q = q + delta_q;
    
    % Increment iteration counter
    iteration = iteration + 1;
    
    % (Optional) You can print the iteration, error, and joint angles for debugging
    % disp(['Iteration: ', num2str(iteration), ' | Error norm: ', num2str(norm(error))])
end

% Store the final joint angles as qGoal
qGoal = q;

% Update the position of the robot in V-REP
updatePos(vrep,connection.clientID,qGoal)

% now disable stepped simulation mode:
simulation_setStepped(connection,false);

pause(5)

% stop the simulation
simulation_stop(connection);

% close the connection
simulation_closeConnection(connection);
