function [ vu, omega ] = calculateControlOutput( robotPose, goalPose, parameters )
%CALCULATECONTROLOUTPUT This function computes the motor velocities for a differential driven robot

% current robot position and orientation
x = robotPose(1);
y = robotPose(2);
theta = robotPose(3);

% goal position and orientation
xg = goalPose(1);
yg = goalPose(2);
thetag = goalPose(3);

% compute control quantities
rho = sqrt((xg - x)^2 + (yg - y)^2);  % Distance to target
lambda = atan2(yg - y, xg - x);       % Angle to target in inertial frame
alpha = normalizeAngle(lambda - theta); % Angle to target in robot frame
beta = normalizeAngle(thetag - lambda); % Orientation error

% Task 3: Basic control law
vu = parameters.Krho * rho; % Proportional control for forward velocity
omega = parameters.Kalpha * alpha + parameters.Kbeta * beta; % Angular velocity control

% Task 4: Handle backward motion and constant speed
if parameters.backwardAllowed && abs(alpha) > pi / 2
    vu = -vu; % Reverse the velocity
    alpha = normalizeAngle(alpha - pi); % Adjust alpha
    beta = normalizeAngle(beta - pi); % Adjust beta
end

if parameters.useConstantSpeed
    if rho > 0 % Avoid division by zero
        scalingFactor = parameters.constantSpeed / abs(vu); % Scale to constant speed
        vu = vu * scalingFactor;
        omega = omega * scalingFactor;
    else
        vu = 0; % Stop when at the goal
        omega = 0;
    end
end
end
