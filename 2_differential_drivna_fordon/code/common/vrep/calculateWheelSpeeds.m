function [ LeftWheelVelocity, RightWheelVelocity ] = calculateWheelSpeeds( vu, omega, parameters )
%CALCULATEWHEELSPEEDS This function computes the motor velocities for a differential driven robot

wheelRadius = parameters.wheelRadius;
halfWheelbase = parameters.interWheelDistance/2;

LeftWheelVelocity = (vu / wheelRadius) - (halfWheelbase * omega / wheelRadius);
RightWheelVelocity = (vu / wheelRadius) + (halfWheelbase * omega / wheelRadius);

end
