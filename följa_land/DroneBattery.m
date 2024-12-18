% MATLAB Script to Control Drone PID Parameters with Switch-Case Structure

% Clear previous connections and variables
sim.simxFinish(-1);
clear all; close all; clc;

% Connect to CoppeliaSim Remote API
sim = remApi('remoteApi');
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

if clientID > -1
    disp('Connected to CoppeliaSim.');

    % Prompt user to select optimization case
    caseChoice = input('Select optimization method: 1 (Regular Optimizer) or 2 (Genetic Algorithm): ');

    switch caseChoice
        case 1 % FIRST CASE - Regular optimizer for PID parameters
            disp('Using Regular Optimizer for PID Parameters...');

            % Optimize PID parameters using the regular optimizer
            [optimalPID, cost] = optimizeDronePID_Regular();

            % Extract optimized PID parameters
            newPParam = optimalPID(1);
            newIParam = optimalPID(2);
            newDParam = optimalPID(3);
            newVParam = optimalPID(4);

            fprintf('Optimal PID Parameters Found: P=%.2f, I=%.2f, D=%.2f, V=%.2f\n', ...
                    newPParam, newIParam, newDParam, newVParam);

        case 2 % SECOND CASE - Genetic Algorithm for PID parameters
            disp('Using Genetic Algorithm for PID Parameters...');

            % Optimize PID parameters using Genetic Algorithm
            [optimalPID, cost] = optimizeDronePID_GA();

            % Extract optimized PID parameters
            newPParam = optimalPID(1);
            newIParam = optimalPID(2);
            newDParam = optimalPID(3);
            newVParam = optimalPID(4);

            fprintf('Optimal PID Parameters Found: P=%.2f, I=%.2f, D=%.2f, V=%.2f\n', ...
                    newPParam, newIParam, newDParam, newVParam);

        otherwise
            disp('Invalid choice. Exiting.');
            sim.simxFinish(clientID);
            sim.delete();
            return;
    end

    % Send the optimized PID parameters to CoppeliaSim
    sim.simxSetFloatSignal(clientID, 'drone_pParam', newPParam, sim.simx_opmode_oneshot);
    sim.simxSetFloatSignal(clientID, 'drone_iParam', newIParam, sim.simx_opmode_oneshot);
    sim.simxSetFloatSignal(clientID, 'drone_dParam', newDParam, sim.simx_opmode_oneshot);
    sim.simxSetFloatSignal(clientID, 'drone_vParam', newVParam, sim.simx_opmode_oneshot);

    % Signal the drone to start
    sim.simxSetIntegerSignal(clientID, 'drone_start', 1, sim.simx_opmode_oneshot);

    % Wait for some time to let the drone perform
    disp('Drone is running...');
    pause(200);

    % Stop the drone
    sim.simxSetIntegerSignal(clientID, 'drone_start', 0, sim.simx_opmode_oneshot);
    disp('Drone stopped.');

    % Stop the simulation
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    disp('Simulation stopped.');

    % Disconnect
    sim.simxFinish(clientID);
    disp('Disconnected from CoppeliaSim.');
else
    disp('Failed to connect to CoppeliaSim.');
end

sim.delete(); % Cleanup

%% Regular Optimizer Function
function [optimalPID, cost] = optimizeDronePID_Regular()
    % Parameters and Initial Conditions
    initialBattery = 100; % Battery starts at 100%
    minBattery = 20;      % Minimum battery level to operate
    timeHorizon = 1000;   % Time for simulation (in seconds)
    dt = 0.1;             % Time step for simulation
    target = [-2.172, 2.137, 0.88]; % Target position (x, y, z)

    % Initial PID Parameters
    initialPID = [1.0, 0.01, 0.10, -1.5]; % [P, I, D, V]

    % Weighting factors
    w1 = 0.1; % Distance weight
    w2 = 0.1; % Control effort weight
    w3 = 1;   % Battery weight

    % Optimization options
    options = optimset('Display', 'iter', 'MaxIter', 100, 'TolFun', 1e-4);

    % Objective function
    objective = @(PID) computeCost(PID, initialBattery, minBattery, target, w1, w2, w3, timeHorizon, dt);

    % Constraints
    lb = [1.0, 0.00, 0.05, -2]; % Lower bounds for P, I, D, V
    ub = [1.4, 0.02, 0.10, 0];  % Upper bounds for P, I, D, V

    % Optimize PID parameters
    [optimalPID, cost] = fmincon(objective, initialPID, [], [], [], [], lb, ub, [], options);
end

%% Genetic Algorithm Optimizer Function
function [optimalPID, cost] = optimizeDronePID_GA()
    % Parameters and Initial Conditions
    initialBattery = 100; % Battery starts at 100%
    minBattery = 20;      % Minimum battery level to operate
    timeHorizon = 1000;   % Time for simulation (in seconds)
    dt = 0.1;             % Time step for simulation
    target = [-2.172, 2.137, 0.88]; % Target position (x, y, z)

    % Weighting factors
    w1 = 0.1; % Distance weight
    w2 = 0.1; % Control effort weight
    w3 = 1.0; % Battery weight

    % Objective function
    objective = @(PID) computeCost(PID, initialBattery, minBattery, target, w1, w2, w3, timeHorizon, dt);

    % Constraints
    lb = [1.0, 0.00, 0.05, -2]; % Lower bounds for P, I, D, V
    ub = [1.4, 0.02, 0.10, 0];  % Upper bounds for P, I, D, V

    % GA Options
    options = optimoptions('ga', ...
        'Display', 'iter', ...
        'MaxGenerations', 100, ...
        'PopulationSize', 50, ...
        'UseParallel', true, ...
        'FunctionTolerance', 1e-6, ...
        'MaxStallGenerations', 30);

    % Run Genetic Algorithm
    [optimalPID, cost, ~, output] = ga(objective, 4, [], [], [], [], lb, ub, [], options);
    disp(output);
end

%% Compute Cost Function
function J = computeCost(PID, initialBattery, minBattery, target, w1, w2, w3, timeHorizon, dt)
    % Unpack PID parameters
    P = PID(1);
    I = PID(2);
    D = PID(3);
    V = PID(4);

    % Initialize state variables
    position = [-0.11915, -0.82466, 0.3158]; % Initial drone position
    velocity = [0, 0, 0]; % Initial drone velocity
    battery = initialBattery;
    cumulError = 0;
    lastError = 0;

    J = 0; % Initialize cost

    for t = 0:dt:timeHorizon
        % Compute error
        error = target - position;

        % PID Control
        controlEffort = abs(P * error) + abs(I * cumulError) + abs(D * (error - lastError) / dt);
        thrust = P * error + I * cumulError + D * (error - lastError) / dt + V * velocity;

        % Update position and velocity (simple dynamics model)
        acceleration = thrust / 1.0; % Assume unit mass
        velocity = velocity + acceleration * dt;
        position = position + velocity * dt;

        % Update battery
        dischargeRate = 0.01 + 0.001 * sum(abs(controlEffort));
        battery = battery - dischargeRate * dt;

        % Update cumulative cost
        distance = norm(target - position);
        J = J + (w1 * distance + w2 * sum(abs(controlEffort)) + w3 / max(battery, minBattery)) * dt;

        % Update cumulative error and last error
        cumulError = cumulError + error * dt;
        lastError = error;

        % Terminate if battery is depleted
        if battery < minBattery
            J = J + 1e6; % Add a high penalty for depleting the battery
            break;
        end
    end
end
