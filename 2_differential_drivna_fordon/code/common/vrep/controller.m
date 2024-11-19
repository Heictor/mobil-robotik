%% V-REP Simulation Exercise 4 and 5: Kinematic Control
% Tests the implemented control algorithm within a V-REP simulation.

clear;
close all; 
%% Parameters setup

%% define that we will use the real P3DX and/or the simulated one
global realRobot ;
realRobot = 0;  % Set to 1 for real robot, 0 for simulation.

global laserStr;
laserStr = '/perception/laser/';

global poseStr;
poseStr = '/motion/pose';   

global vel2Str;
vel2Str = '/motion/vel2';   

global stopStr;
stopStr = '/motion/stop';

global parameters;

%% Initialization
if realRobot == 1
    % Real robot initialization
    http_init;

    % Declaration of variables
    connection = 'http://10.1.3.130:4950';
    parameters.wheelDiameter = 0.195;
    parameters.wheelRadius = parameters.wheelDiameter / 2.0;
    parameters.interWheelDistance = 0.381 / 2.0;
else
    % V-REP simulation initialization
    connection = simulation_setup();
    connection = simulation_openConnection(connection, 0);
    simulation_start(connection);

    % Get static data from V-REP
    Pioneer_p3dx_init(connection);
    parameters.wheelDiameter = Pioneer_p3dx_getWheelDiameter(connection);
    parameters.wheelRadius = parameters.wheelDiameter / 2.0;
    parameters.interWheelDistance = Pioneer_p3dx_getInterWheelDistance(connection);
    parameters.scannerPoseWrtPioneer_p3dx = Pioneer_p3dx_getScannerPose(connection);
    Pioneer_p3dx_setTargetGhostVisible(connection, 1);
end

Pioneer_p3dx_setPose(connection, 0, 0, 0);

% Define target position
Pioneer_p3dx_setTargetGhostPose(connection, 1.6, 0.6, deg2rad(90));

% Controller parameters
parameters.Krho = 0.5;
parameters.Kalpha = 1.5;
parameters.Kbeta = -0.6;
parameters.backwardAllowed = true; % Enable backward motion
parameters.useConstantSpeed = true; % Enable constant speed control
parameters.constantSpeed = 0.1; % Desired constant speed [m/s]

% Thresholds
parameters.dist_threshold = 0.25; % Distance threshold to goal
parameters.angle_threshold = deg2rad(10); % Orientation threshold to goal

%% Control Loop
EndCond = 0;

while (~EndCond)
    %% Control Step
    % Get pose and goalPose from V-REP
    [x, y, theta] = Pioneer_p3dx_getPose(connection);
    [xg, yg, thetag] = Pioneer_p3dx_getTargetGhostPose(connection);

    % Run control step
    [vu, omega] = calculateControlOutput([x, y, theta], [xg, yg, thetag], parameters);

    % Calculate wheel speeds
    [LeftWheelVelocity, RightWheelVelocity] = calculateWheelSpeeds(vu, omega, parameters);

    % Calculate end condition
    dtheta = abs(normalizeAngle(theta - thetag));
    rho = sqrt((xg - x)^2 + (yg - y)^2);  % Distance to goal
    EndCond = (rho < parameters.dist_threshold && dtheta < parameters.angle_threshold);     

    % Set robot wheel speeds
    Pioneer_p3dx_setWheelSpeeds(connection, LeftWheelVelocity, RightWheelVelocity);
end

%% Bring Pioneer_p3dx to standstill
Pioneer_p3dx_setWheelSpeeds(connection, 0.0, 0.0);

% Stop simulation or close connection
if realRobot ~= 1
    simulation_stop(connection);
    simulation_closeConnection(connection);
end
disp('Simulation ended.');
