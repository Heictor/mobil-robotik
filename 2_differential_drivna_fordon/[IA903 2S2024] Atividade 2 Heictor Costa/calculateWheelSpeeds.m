function [ LeftWheelVelocity, RightWheelVelocity ] = calculateWheelSpeeds( vu, omega, parameters )
%CALCULATEWHEELSPEEDS This function computes the motor velocities for a differential-driven robot.

    % Extract parameters
    wheelRadius = parameters.wheelRadius;
    halfWheelbase = parameters.interWheelDistance / 2;

    % Compute wheel velocities based on the provided kinematic model
    LeftWheelVelocity = (vu - omega * halfWheelbase) / wheelRadius;
    RightWheelVelocity = (vu + omega * halfWheelbase) / wheelRadius;

end
