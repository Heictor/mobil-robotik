% Define constants and parameters
safeVerticalOffset = 2.0; % Safe vertical offset from target
safeHorizontalDistance = 1.0; % Minimum horizontal distance for corrections
stationaryTimeLimit = 5.0; % Time threshold for landing
weights = [1, 1, 1, 0.5]; % Weights for [Vertical, Horizontal, Rotational, Stability errors]

% Define target and drone properties
targetPosition = [10, 10, 5]; % Example target position (x, y, z)
initialDronePosition = [0, 0, 0]; % Initial drone position (x, y, z)
droneOrientation = 0; % Initial drone yaw (orientation)

% Genetic Algorithm Optimization
options = optimoptions('ga', 'Display', 'iter', 'PopulationSize', 50, ...
                       'MaxGenerations', 100, 'FunctionTolerance', 1e-6);

% Define bounds for PID parameters
lb = [0.1, 0, 0, 0.1, 0, 0.1, 0]; % Lower bounds: [Pz, Iz, Dz, Pxy, Dxy, Pyaw, Dyaw]
ub = [5, 2, 2, 5, 2, 5, 2];       % Upper bounds: [Pz, Iz, Dz, Pxy, Dxy, Pyaw, Dyaw]

% Run Genetic Algorithm
[optimalParams, optimalCost] = ga(@(params) objectiveFunction(params, ...
    targetPosition, initialDronePosition, droneOrientation, ...
    safeVerticalOffset, safeHorizontalDistance, stationaryTimeLimit, weights), ...
    7, [], [], [], [], lb, ub, [], options);

% Display results
fprintf('Optimal PID Parameters:\n');
fprintf('Pz = %.3f, Iz = %.3f, Dz = %.3f\n', optimalParams(1), optimalParams(2), optimalParams(3));
fprintf('Pxy = %.3f, Dxy = %.3f\n', optimalParams(4), optimalParams(5));
fprintf('Pyaw = %.3f, Dyaw = %.3f\n', optimalParams(6), optimalParams(7));
fprintf('Optimal Cost (Objective Function): %.5f\n', optimalCost);

% Objective Function
function J = objectiveFunction(params, targetPosition, initialDronePosition, ...
    droneOrientation, safeVerticalOffset, safeHorizontalDistance, ...
    stationaryTimeLimit, weights)

    % Unpack PID parameters
    Pz = params(1); Iz = params(2); Dz = params(3);
    Pxy = params(4); Dxy = params(5);
    Pyaw = params(6); Dyaw = params(7);

    % Initialize errors
    verticalError = 0;
    horizontalError = 0;
    rotationalError = 0;
    stabilityError = 0;
    
    % Simulation parameters
    simulationTime = 10; % seconds
    dt = 0.1; % time step (s)
    dronePosition = initialDronePosition; % Start with initial position
    cumulativeVerticalError = 0;
    prevVerticalError = 0;
    prevHorizontalError = [0, 0];
    prevYawError = 0;
    velocities = []; % Store velocity changes for stability calculation
    
    for t = 0:dt:simulationTime
        % Target position with vertical offset
        targetWithOffset = [targetPosition(1), targetPosition(2), targetPosition(3) + safeVerticalOffset];
        
        % Vertical control
        errorZ = targetWithOffset(3) - dronePosition(3); % Vertical error
        cumulativeVerticalError = cumulativeVerticalError + errorZ * dt;
        verticalControl = Pz * errorZ + Iz * cumulativeVerticalError + Dz * (errorZ - prevVerticalError) / dt;
        prevVerticalError = errorZ;
        
        % Horizontal control
        errorXY = targetPosition(1:2) - dronePosition(1:2); % Horizontal error (x, y)
        horizontalDistance = norm(errorXY); % Euclidean distance
        horizontalControl = Pxy * errorXY + Dxy * (errorXY - prevHorizontalError) / dt;
        prevHorizontalError = errorXY;

        % Rotational control
        yawError = 0 - droneOrientation; % Assume target orientation = 0
        rotationalControl = Pyaw * yawError + Dyaw * (yawError - prevYawError) / dt;
        prevYawError = yawError;

        % Update drone position (simple simulation)
        dronePosition(1) = dronePosition(1) + horizontalControl(1) * dt;
        dronePosition(2) = dronePosition(2) + horizontalControl(2) * dt;
        dronePosition(3) = dronePosition(3) + verticalControl * dt;
        droneOrientation = droneOrientation + rotationalControl * dt;

        % Store velocity changes for stability analysis
        velocities = [velocities; verticalControl, horizontalControl, rotationalControl];
        
        % Update errors
        verticalError = verticalError + errorZ^2 * dt;
        horizontalError = horizontalError + horizontalDistance^2 * dt;
        rotationalError = rotationalError + yawError^2 * dt;
    end

    % Stability metric (penalizes abrupt velocity changes)
    stabilityError = sum(diff(velocities).^2, 'all');

    % Objective function
    J = weights(1) * verticalError + weights(2) * horizontalError + ...
        weights(3) * rotationalError + weights(4) * stabilityError;
end

sim.simxCallScriptFunction(clientID, 'Quadcopter', sim.sim_scripttype_childscript, 'remoteApi_setParameters', [], [], [], {}, sim.simx_opmode_blocking);
sim.simxCallScriptFunction(clientID, 'OmniPlatform', sim.sim_scripttype_childscript, 'remoteApi_setParameters', [], [], [], {}, sim.simx_opmode_blocking);


%simpleTest()
setDroneParametersInCoppelia()
function setDroneParametersInCoppelia()
    % Initialize parameters for GA optimization
    disp('Connecting to CoppeliaSim...');
    sim = remApi('remoteApi'); % Import remote API library
    sim.simxFinish(-1); % Close any previous connections
    clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5); % Connect to CoppeliaSim

    if clientID > -1
        disp('Connected to CoppeliaSim.');

        % Example optimized PID parameters from GA (replace with actual results)
        optimalParams = [1.6, 0.1, 0.01, 1.2, 0.05, 0.8, 0.02]; 
        % [Pz, Iz, Dz, Pxy, Dxy, Pyaw, Dyaw]

        % Mapping parameters to Lua script
        % Create a Lua command string with the optimal parameters
        luaCommand = sprintf([...
            'pParam=%f; '... % Pz
            'iParam=%f; '... % Iz
            'dParam=%f; '... % Dz
            'pParamXY=%f; '... % Pxy
            'dParamXY=%f; '... % Dxy
            'pParamYaw=%f; '... % Pyaw
            'dParamYaw=%f; '... % Dyaw
            'print("PID parameters updated in Lua.")'], ...
            optimalParams(1), optimalParams(2), optimalParams(3), ...
            optimalParams(4), optimalParams(5), ...
            optimalParams(6), optimalParams(7));

        sim.simxCallScriptFunction(clientID, 'Quadcopter', sim.sim_scripttype_childscript, 'remoteApi_setParameters', [], [], [], {}, sim.simx_opmode_blocking);
        sim.simxCallScriptFunction(clientID, 'OmniPlatform', sim.sim_scripttype_childscript, 'remoteApi_setParameters', [], [], [], {}, sim.simx_opmode_blocking);

        % Execute Lua script in CoppeliaSim
        [returnCode] = sim.simxCallScriptFunction(clientID, ...
            'Quadcopter', ... % Quadcopter Lua script handle
            sim.sim_scripttype_childscript, ...
            'remoteApi_setParameters', ... % Lua function to handle parameters
            [], [], [], luaCommand, ...
            sim.simx_opmode_blocking);

        if returnCode == sim.simx_return_ok
            disp('Drone PID parameters successfully updated in CoppeliaSim.');
        else
            disp('Failed to update PID parameters in CoppeliaSim.');
        end

        % Close the connection
        sim.simxFinish(clientID);
    else
        disp('Failed to connect to CoppeliaSim.');
    end

    sim.delete(); % Cleanup API
    disp('Connection to CoppeliaSim closed.');
end
