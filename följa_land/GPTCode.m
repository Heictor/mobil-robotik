% % MATLAB Script to Control Drone PID Parameters

% % sim.simxFinish(clientID)
% % clear all; close all; clc;
% % 
% % % Connect to CoppeliaSim Remote API
% % sim = remApi('remoteApi');
% % clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5)
% % 
% % if clientID > -1
% %     disp('Connected to CoppeliaSim.');
% % 
% %     % Define new PID parameters
% %     newPParam = 1.5;
% %     newIParam = 0.0;
% %     newDParam = 0.0;
% %     newVParam = -2;
% %     % Send the PID parameters to CoppeliaSim
% %     sim.simxSetFloatSignal(clientID, 'drone_pParam', newPParam, sim.simx_opmode_oneshot);
% %     sim.simxSetFloatSignal(clientID, 'drone_iParam', newIParam, sim.simx_opmode_oneshot);
% %     sim.simxSetFloatSignal(clientID, 'drone_dParam', newDParam, sim.simx_opmode_oneshot);
% %     sim.simxSetFloatSignal(clientID, 'drone_vParam', newVParam, sim.simx_opmode_oneshot);
% % 
% %     % Signal the drone to start
% %     sim.simxSetIntegerSignal(clientID, 'drone_start', 1, sim.simx_opmode_oneshot);
% % 
% %     % Wait for some time to let the drone perform
% %     pause(20);
% % 
% %     % Stop the drone
% %     sim.simxSetIntegerSignal(clientID, 'drone_start', 0, sim.simx_opmode_oneshot);
% % 
% %     % Disconnect
% %     sim.simxFinish(clientID);
% %     disp('Simulation completed and disconnected.');
% % else
% %     disp('Failed to connect to CoppeliaSim.');
% % end
% % 
% % sim.delete(); % Cleanup
% % 
% % % Stop the simulation
% % sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
% % 
% % % Close the connection to the simulator
% % sim.simxFinish(clientID);

% % %MATLAB Script to Optimize and Control Drone PID Parameters in CoppeliaSim
% % sim.simxFinish(clientID)
% % clear all; close all; clc;
% % 
% % % Connect to CoppeliaSim Remote API
% % sim = remApi('remoteApi');
% % clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5)
% % 
% % if clientID > -1
% %     disp('Connected to CoppeliaSim.');
% % 
% %     % Optimize PID parameters using the defined MATLAB function
% %     [optimalPID, cost] = optimizeDronePID();
% % 
% %     % Extract optimized PID parameters
% %     newPParam = optimalPID(1);
% %     newIParam = optimalPID(2);
% %     newDParam = optimalPID(3);
% %     newVParam = optimalPID(4);
% % 
% %     fprintf('Optimal PID Parameters Found: P=%.2f, I=%.2f, D=%.2f, V=%.2f\n', ...
% %             newPParam, newIParam, newDParam, newVParam);
% % 
% %     % Send the optimized PID parameters to CoppeliaSim
% %     sim.simxSetFloatSignal(clientID, 'drone_pParam', newPParam, sim.simx_opmode_oneshot);
% %     sim.simxSetFloatSignal(clientID, 'drone_iParam', newIParam, sim.simx_opmode_oneshot);
% %     sim.simxSetFloatSignal(clientID, 'drone_dParam', newDParam, sim.simx_opmode_oneshot);
% %     sim.simxSetFloatSignal(clientID, 'drone_vParam', newVParam, sim.simx_opmode_oneshot);
% % 
% %     % Define parameters for monitoring
% %     samplingInterval = 0.1; % Sampling interval in seconds
% %     simulationDuration = 1000; % Total simulation duration in seconds
% %     numSamples = simulationDuration / samplingInterval; % Number of samples
% % 
% %     % Initialize storage for battery data
% %     batteryLevels = zeros(1, numSamples); % Battery levels
% %     dischargeRates = zeros(1, numSamples); % Battery discharge rates
% %     timeStamps = linspace(0, simulationDuration, numSamples); % Time vector
% % 
% %     % Signal the drone to start
% %     sim.simxSetIntegerSignal(clientID, 'drone_start', 1, sim.simx_opmode_oneshot);
% % 
% %     % Loop to collect battery data
% %     for i = 1:numSamples
% %         % Retrieve battery level from CoppeliaSim
% %         [~, batteryLevel] = sim.simxGetFloatSignal(clientID, 'drone_batteryLevel', sim.simx_opmode_blocking);
% % 
% %         % Compute discharge rate using helper function
% %         dischargeRate = computeDischargeRate(batteryLevels, batteryLevel, i, samplingInterval);
% % 
% %         % Store battery level and discharge rate
% %         batteryLevels(i) = batteryLevel;
% %         dischargeRates(i) = dischargeRate;
% % 
% %         % Pause for the sampling interval
% %         disp('Drone is running...')
% %         disp(i)
% %         pause(samplingInterval);
% %     end  
% % 
% %     % Wait for some time to let the drone perform
% %     % disp('Drone is running...');
% %     % pause(90);
% % 
% %     % Stop the drone
% %     sim.simxSetIntegerSignal(clientID, 'drone_start', 0, sim.simx_opmode_oneshot);
% %     disp('Drone stopped.');
% % 
% %     % Stop the simulation
% %     sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
% %     disp('Simulation stopped.');
% % 
% %     % Disconnect
% %     sim.simxFinish(clientID);
% %     disp('Disconnected from CoppeliaSim.');
% % else
% %     disp('Failed to connect to CoppeliaSim.');
% % end
% % 
% % sim.delete(); % Cleanup
% % 
% % % Plot battery data
% % figure;
% % hold on;
% % plot(timeStamps, batteryLevels, '-b', 'LineWidth', 1.5, 'DisplayName', 'Battery Level (%)');
% % plot(timeStamps, dischargeRates, '-r', 'LineWidth', 1.5, 'DisplayName', 'Battery Discharge Rate (%/s)');
% % xlabel('Time (s)');
% % ylabel('Battery Level / Discharge Rate');
% % title('Drone Battery Level and Discharge Rate Over Time');
% % legend('show');
% % grid on;
% % hold off;
% % 
% % % Function to Optimize Drone PID Parameters
% % function [optimalPID, cost] = optimizeDronePID()
% %     % Parameters and Initial Conditions
% %     initialBattery = 100; % Battery starts at 100%
% %     minBattery = 20;      % Minimum battery level to operate
% %     timeHorizon = 1000;     % Time for simulation (in seconds)
% %     dt = 0.1;             % Time step for simulation
% %     target = [-2.172, 2.137, 0.88];   % Target position (x, y, z)
% % 
% %     % Initial PID Parameters
% %     initialPID = [1.0, 0.01, 0.10, -1.5]; % [P, I, D, V]
% % 
% %     % Weighting factors
% %     w1 = 0.5; % Distance weight
% %     w2 = 1.0; % Control effort weight
% %     w3 = 0.1; % Battery weight
% % 
% %     % Optimization options
% %     options = optimset('Display', 'iter', 'MaxIter', 100, 'TolFun', 1e-4);
% % 
% %     % Objective function
% %     objective = @(PID) computeCost(PID, initialBattery, minBattery, target, w1, w2, w3, timeHorizon, dt);
% % 
% %     % Constraints
% %     lb = [0.9, 0.00, 0.09, -1.6]; % Lower bounds for P, I, D, V
% %     ub = [1.1, 0.02, 0.11, -1.4]; % Upper bounds for P, I, D, V
% % 
% %     % Optimize PID parameters
% %     [optimalPID, cost] = fmincon(objective, initialPID, [], [], [], [], lb, ub, [], options);
% % end
% % 
% % function J = computeCost(PID, initialBattery, minBattery, target, w1, w2, w3, timeHorizon, dt)
% %     % Unpack PID parameters
% %     P = PID(1);
% %     I = PID(2);
% %     D = PID(3);
% %     V = PID(4);
% % 
% %     % Initialize state variables
% %     position = [0, 0, 0]; % Initial drone position
% %     velocity = [0, 0, 0]; % Initial drone velocity
% %     battery = initialBattery;
% %     cumulError = 0;
% %     lastError = 0;
% % 
% %     J = 0; % Initialize cost
% % 
% %     for t = 0:dt:timeHorizon
% %         % Compute error
% %         error = target - position;
% % 
% %         % PID Control
% %         controlEffort = abs(P * error) + abs(I * cumulError) + abs(D * (error - lastError) / dt);
% %         thrust = P * error + I * cumulError + D * (error - lastError) / dt + V * velocity;
% % 
% %         % Update position and velocity (simple dynamics model)
% %         acceleration = thrust / 1.0; % Assume unit mass
% %         velocity = velocity + acceleration * dt;
% %         position = position + velocity * dt;
% % 
% %         % Update battery
% %         dischargeRate = computeBatteryDischarge(controlEffort);%0.01 + 0.001 * sum(abs(controlEffort));
% %         battery = battery - dischargeRate * dt;
% % 
% %         % Update cumulative cost
% %         distance = norm(target - position);
% %         J = J + (w1 * distance + w2 * sum(abs(controlEffort)) + w3 / max(battery, minBattery)) * dt;
% % 
% %         % Update cumulative error and last error
% %         cumulError = cumulError + error * dt;
% %         lastError = error;
% % 
% %         % Terminate if battery is depleted
% %         if battery < minBattery
% %             J = J + 1e6; % Add a high penalty for depleting the battery
% %             break;
% %         end
% %     end
% % end
% % 
% % function dischargeRate = computeBatteryDischarge(controlEffort)
% %     % Compute battery discharge rate based on control effort
% %     dischargeRate = 0.01 + 0.001 * sum(abs(controlEffort));
% % end
% % 
% % function dischargeRate = computeDischargeRate(batteryLevels, batteryLevel, i, samplingInterval)
% %     % Compute discharge rate from battery levels
% %     if i == 1
% %         dischargeRate = 0; % No discharge rate for the first sample
% %     else
% %         dischargeRate = abs(batteryLevels(i - 1) - batteryLevel) / samplingInterval;
% %     end
% % end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Smooth

% % MATLAB Script to Optimize and Control Drone PID Parameters in CoppeliaSim
%sim.simxFinish(clientID)
%sim.simxFinish(clientID)
clear all; close all; clc;

% Connect to CoppeliaSim Remote API
sim = remApi('remoteApi');
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5)

if clientID > -1
    disp('Connected to CoppeliaSim.');

    % Optimize PID parameters using the defined MATLAB function
    [optimalPID, cost] = optimizeDronePID();

    % Extract optimized PID parameters
    newPParam = optimalPID(1);
    newIParam = optimalPID(2);
    newDParam = optimalPID(3);
    newVParam = optimalPID(4);

    fprintf('Optimal PID Parameters Found: P=%.2f, I=%.2f, D=%.2f, V=%.2f\n', ...
            newPParam, newIParam, newDParam, newVParam);

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

% Function to Optimize Drone PID Parameters
function [optimalPID, cost] = optimizeDronePID()
    % Parameters and Initial Conditions
    initialBattery = 100; % Battery starts at 100%
    minBattery = 20;      % Minimum battery level to operate
    timeHorizon = 1000;     % Time for simulation (in seconds)
    dt = 0.1;             % Time step for simulation
    target = [-2.172, 2.137, 0.88];   % Target position (x, y, z)

    % Initial PID Parameters
    initialPID = [1.0, 0.01, 0.10, -1.5]; % [P, I, D, V]

    % Weighting factors
    w1 = 0.1; % Distance weight
    w2 = 0.1; % Control effort weight
    w3 = 1; % Battery weight

    % Optimization options
    options = optimset('Display', 'iter', 'MaxIter', 100, 'TolFun', 1e-4);

    % Objective function
    objective = @(PID) computeCost(PID, initialBattery, minBattery, target, w1, w2, w3, timeHorizon, dt);

    % Constraints
    lb = [1.0, 0.00, 0.05, -2]; % Lower bounds for P, I, D, V
    ub = [1.4, 0.02, 0.10, 0]; % Upper bounds for P, I, D, V

    % Optimize PID parameters
    [optimalPID, cost] = fmincon(objective, initialPID, [], [], [], [], lb, ub, [], options);
end

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
        % disp(sum(controlEffort))
        thrust = P * error + I * cumulError + D * (error - lastError) / dt + V * velocity;

        % Update position and velocity (simple dynamics model)
        acceleration = thrust / 1.0; % Assume unit mass
        velocity = velocity + acceleration * dt;
        position = position + velocity * dt;

        % Update battery
        dischargeRate = 0.1 * sum(abs(controlEffort));
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% GA Smooth
% MATLAB Script to Optimize and Control Drone PID Parameters in CoppeliaSim
% sim.simxFinish(clientID)
% clear all; close all; clc;
% 
% % Connect to CoppeliaSim Remote API
% sim = remApi('remoteApi');
% clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
% 
% if clientID > -1
%     disp('Connected to CoppeliaSim.');
% 
%     % Optimize PID parameters using Genetic Algorithm
%     [optimalPID, cost] = optimizeDronePID();
% 
%     % Extract optimized PID parameters
%     newPParam = optimalPID(1);
%     newIParam = optimalPID(2);
%     newDParam = optimalPID(3);
%     newVParam = optimalPID(4);
% 
%     fprintf('Optimal PID Parameters Found: P=%.2f, I=%.2f, D=%.2f, V=%.2f\n', ...
%             newPParam, newIParam, newDParam, newVParam);
% 
%     % Send the optimized PID parameters to CoppeliaSim
%     sim.simxSetFloatSignal(clientID, 'drone_pParam', newPParam, sim.simx_opmode_oneshot);
%     sim.simxSetFloatSignal(clientID, 'drone_iParam', newIParam, sim.simx_opmode_oneshot);
%     sim.simxSetFloatSignal(clientID, 'drone_dParam', newDParam, sim.simx_opmode_oneshot);
%     sim.simxSetFloatSignal(clientID, 'drone_vParam', newVParam, sim.simx_opmode_oneshot);
% 
%     % Signal the drone to start
%     sim.simxSetIntegerSignal(clientID, 'drone_start', 1, sim.simx_opmode_oneshot);
% 
%     % Wait for some time to let the drone perform
%     disp('Drone is running...');
%     pause(200);
% 
%     % Stop the drone
%     sim.simxSetIntegerSignal(clientID, 'drone_start', 0, sim.simx_opmode_oneshot);
%     disp('Drone stopped.');
% 
%     % Stop the simulation
%     sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
%     disp('Simulation stopped.');
% 
%     % Disconnect
%     sim.simxFinish(clientID);
%     disp('Disconnected from CoppeliaSim.');
% else
%     disp('Failed to connect to CoppeliaSim.');
% end
% 
% sim.delete(); % Cleanup
% 
% % Function to Optimize Drone PID Parameters
% function [optimalPID, cost] = optimizeDronePID()
%     % Parameters and Initial Conditions
%     initialBattery = 100; % Battery starts at 100%
%     minBattery = 20;      % Minimum battery level to operate
%     timeHorizon = 1000;   % Time for simulation (in seconds)
%     dt = 0.1;             % Time step for simulation
%     target = [-2.172, 2.137, 0.88]; % Target position (x, y, z)
% 
%     % Weighting factors
%     w1 = 0.1; % Distance weight
%     w2 = 0.1; % Control effort weight
%     w3 = 1.0; % Battery weight
% 
%     % Objective function
%     objective = @(PID) computeCost(PID, initialBattery, minBattery, target, w1, w2, w3, timeHorizon, dt);
% 
%     % Constraints
%       lb = [1.0, 0.00, 0.05, -2]; % Lower bounds for P, I, D, V
%       ub = [1.4, 0.02, 0.10, 0]; % Upper bounds for P, I, D, V
% 
%     % GA Options
%     options = optimoptions('ga', ...
%         'Display', 'iter', ...
%         'MaxGenerations', 100, ...
%         'PopulationSize', 50, ...
%         'UseParallel', true,...
%         'FunctionTolerance',1e-6,...
%         'MaxStallGenerations',30);
% 
%     % Run Genetic Algorithm
%     [optimalPID, cost, ~, output] = ga(objective, 4, [], [], [], [], lb, ub, [], options);
%     disp(output)
% end
% 
% function J = computeCost(PID, initialBattery, minBattery, target, w1, w2, w3, timeHorizon, dt)
%     % Unpack PID parameters
%     P = PID(1);
%     I = PID(2);
%     D = PID(3);
%     V = PID(4);
% 
%     % Initialize state variables
%     position = [-0.11915, -0.82466, 0.3158]; % Initial drone position
%     velocity = [0, 0, 0]; % Initial drone velocity
%     battery = initialBattery;
%     cumulError = 0;
%     lastError = 0;
% 
%     J = 0; % Initialize cost
% 
%     for t = 0:dt:timeHorizon
%         % Compute error
%         error = target - position;
% 
%         % PID Control
%         controlEffort = abs(P * error) + abs(I * cumulError) + abs(D * (error - lastError) / dt);
%         thrust = P * error + I * cumulError + D * (error - lastError) / dt + V * velocity;
% 
%         % Update position and velocity (simple dynamics model)
%         acceleration = thrust / 1.0; % Assume unit mass
%         velocity = velocity + acceleration * dt;
%         position = position + velocity * dt;
% 
%         % Update battery
%         dischargeRate = 0.01 + 0.001 * sum(abs(controlEffort));
%         battery = battery - dischargeRate * dt;
% 
%         % Update cumulative cost
%         distance = norm(target - position);
%         J = J + (w1 * distance + w2 * sum(abs(controlEffort)) + w3 / max(battery, minBattery)) * dt;
% 
%         % Update cumulative error and last error
%         cumulError = cumulError + error * dt;
%         lastError = error;
% 
%         % Terminate if battery is depleted
%         if battery < minBattery
%             J = J + 1e6; % Add a high penalty for depleting the battery
%             break;
%         end
%     end
% end
