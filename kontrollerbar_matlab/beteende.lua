function sysCall_init()
    -- Inicializa os manipuladores dos motores e do sensor
    leftMotorHandle = sim.getObject("./leftMotor")
    rightMotorHandle = sim.getObject("./rightMotor")
    proximitySensorHandle = sim.getObject("./Proximity_sensor")
    
    print("left motor handle: ", leftMotorHandle, " right motor handle: ", rightMotorHandle, " Proximity_sensor handle: ", proximitySensorHandle)
    
    -- Define as vari?veis de convers?o
    deg2rad = math.pi / 180
    rad2deg = 180 / math.pi

    -- Inicializa a dire??o dos motores
    moveForwardSpeed = 0 * deg2rad
    moveBackwardSpeed = 0 * deg2rad
    turnSpeed = 40 * deg2rad -- Ajuste a velocidade de rota??o conforme necess?rio

    -- Inicializa o estado do robot
    isAvoidingObstacle = false
    obstacleDetectionDistance = 0.34 -- Dist?ncia m?nima para detectar um obst?culo (em metros)
end

function sysCall_actuation()
    -- Read os valores do sensor de proximidade
    local detect, dist = sim.readProximitySensor(proximitySensorHandle)
    print("detect ", detect, " dist ", dist)
    

    
    message,auxiliaryData=sim.getSimulatorMessage()
    if(message==sim.message_keypress) then
        if auxiliaryData[1]==string.byte('w') then
            moveForwardSpeed = 90 * deg2rad
            sim.setJointTargetVelocity(leftMotorHandle, moveForwardSpeed)
            sim.setJointTargetVelocity(rightMotorHandle, moveForwardSpeed)
            
        end
        if auxiliaryData[1]==string.byte('s') then
            moveForwardSpeed = -90 * deg2rad
            sim.setJointTargetVelocity(leftMotorHandle, moveForwardSpeed)
            sim.setJointTargetVelocity(rightMotorHandle, moveForwardSpeed)            
            
        end        
        if auxiliaryData[1]==string.byte('k') then
            sim.setJointTargetVelocity(leftMotorHandle, -moveForwardSpeed)
            sim.setJointTargetVelocity(rightMotorHandle, moveForwardSpeed)
            
            
        end
        if auxiliaryData[1]==string.byte('l') then
            sim.setJointTargetVelocity(leftMotorHandle, moveForwardSpeed)
            sim.setJointTargetVelocity(rightMotorHandle, -moveForwardSpeed)        
            
        end
        if auxiliaryData[1]==string.byte(' ') then
            moveForwardSpeed = 0 * deg2rad        
            sim.setJointTargetVelocity(leftMotorHandle, moveForwardSpeed)
            sim.setJointTargetVelocity(rightMotorHandle, moveForwardSpeed)        
            omega=0
            VL=0
            
        end        
    
    
    
    end
    
    
    
     -- Obt?m as velocidades atuais dos motores
    local leftMotorVelocity = sim.getJointVelocity(leftMotorHandle)
    local rightMotorVelocity = sim.getJointVelocity(rightMotorHandle)

    -- Exibe a velocidade das rodas
    print("Left motor velocity: ", leftMotorVelocity)
    print("Right motor velocity: ", rightMotorVelocity)


end




function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end



-- See the user manual or the available code snippets for additional callback functions and details
