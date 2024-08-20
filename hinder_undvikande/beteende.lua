function sysCall_init()
    -- Initialize the motors and the sensor handles
    leftMotorHandle = sim.getObject("./leftMotor")
    rightMotorHandle = sim.getObject("./rightMotor")
    proximitySensorHandle = sim.getObject("./Proximity_sensor")
    
    print("left motor handle: ", leftMotorHandle, " right motor handle: ", rightMotorHandle, " Proximity_sensor handle: ", proximitySensorHandle)
    
    -- Setting conversion variables
    deg2rad = math.pi / 180
    rad2deg = 180 / math.pi

    -- Initialize motors direcition
    moveForwardSpeed = 90 * deg2rad
    moveBackwardSpeed = -90 * deg2rad
    turnSpeed = 40 * deg2rad -- Adjust rotation velocity as needed

    -- Initialize the robot state
    isAvoidingObstacle = false
    obstacleDetectionDistance = 0.34 -- Minimal detection distance (in meters)
end

function sysCall_actuation()
    -- Read the proximity sensor values
    local detect, dist = sim.readProximitySensor(proximitySensorHandle)
    print("detect ", detect, " dist ", dist)
    
     -- Get the current motor speed
    local leftMotorVelocity = sim.getJointVelocity(leftMotorHandle)
    local rightMotorVelocity = sim.getJointVelocity(rightMotorHandle)

    -- Print the current motor speed
    print("Left motor velocity: ", leftMotorVelocity)
    print("Right motor velocity: ", rightMotorVelocity)

    -- Scans for nearby obstacles
    if detect > 0 and dist < obstacleDetectionDistance then
        -- If nearby object, change direction
        if not isAvoidingObstacle then
            print("Obstacle detected! Changing direction...")
            isAvoidingObstacle = true
            
            -- Rotate robot to the left
            sim.setJointTargetVelocity(leftMotorHandle, -turnSpeed)
            sim.setJointTargetVelocity(rightMotorHandle,turnSpeed)
        end
    else
        -- If no obstacle, maintain direction
        if isAvoidingObstacle then
            print("Obstacle cleared. Moving forward...")
            isAvoidingObstacle = false
            
            -- Go back to forward movement
            sim.setJointTargetVelocity(leftMotorHandle, moveForwardSpeed)
            sim.setJointTargetVelocity(rightMotorHandle, moveForwardSpeed)
        elseif not isAvoidingObstacle then
            -- Keep forward movement
            sim.setJointTargetVelocity(leftMotorHandle, moveForwardSpeed)
            sim.setJointTargetVelocity(rightMotorHandle, moveForwardSpeed)
        end
    end
end


function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end



-- See the user manual or the available code snippets for additional callback functions and details
