-- Generate a sample from a Gaussian distribution
function gaussian (mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random())) *
            math.cos(2 * math.pi * math.random()) + mean
end

function createRandomBumpyFloor()
    print ("Generating new random bumpy floor.")
    sim.setThreadAutomaticSwitch(false)

    -- Remove existing bumpy floor if there already is one
    if (heightField ~= nil) then
        sim.setObjectPosition(heightField, heightField, {0.05, 0, 0})
        return
    end
    --  Create random bumpy floor for robot to drive on
    floorSize = 5
    --heightFieldResolution = 0.3
    --heightFieldNoise = 0.00000005
    heightFieldResolution = 0.1
    heightFieldNoise = 0.0000008
    cellsPerSide = floorSize / heightFieldResolution
    cellHeights = {}
    for i=1,cellsPerSide*cellsPerSide,1 do
        table.insert(cellHeights, gaussian(0, heightFieldNoise))
    end
    heightField=sim.createHeightfieldShape(0, 0, cellsPerSide, cellsPerSide, floorSize, cellHeights)
    -- Make the floor invisible
    sim.setObjectInt32Param(heightField,10,0)
    sim.setThreadAutomaticSwitch(true)
end



-- This function is executed exactly once when the scene is initialised
function sysCall_init()

    tt = sim.getSimulationTime()
    print("Init hello", tt)
          
    robotBase=sim.getObjectHandle(sim.handle_self) -- robot handle
    leftMotor=sim.getObjectHandle("leftMotor") -- Handle of the left motor
    rightMotor=sim.getObjectHandle("rightMotor") -- Handle of the right motor
    turretMotor=sim.getObjectHandle("turretMotor") -- Handle of the turret motor
    turretSensor=sim.getObjectHandle("turretSensor")
 
    -- Create bumpy floor for robot to drive on
    createRandomBumpyFloor()

   
    -- Usual rotation rate for wheels (radians per second)
    speedBase = 5
    speedBaseL = 0
    speedBaseR = 0
    
    -- Which step are we in?
    -- 0 is a dummy value which is immediately completed
    stepCounter = 0
    stepCompletedFlag = false
    stepList = {}
        

    stepList[1] = {"forward", 1.0}
    stepList[2] = {"stop"}
    stepList[3] = {"turn", math.rad(90)}
    stepList[4] = {"stop"}
    stepList[5] = {"repeat"}


 
 
    -- Create and initialise arrays for particles, and display them with dummies
    xArray = {}
    yArray = {}
    thetaArray = {}
    weightArray = {}
    dummyArray = {}
    N = 100
    for i=1, N do
        xArray[i] = 0
        yArray[i] = 0
        thetaArray[i] = 0
        weightArray[i] = 1/N
        dummyArray[i] = sim.createDummy(0.05)
        sim.setObjectPosition(dummyArray[i], -1, {0,0,0})
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,0})
    end

 

    -- Target positions for joints
    motorAngleTargetL = 0.0
    motorAngleTargetR = 0.0

     -- To calibrate
    motorAnglePerMetre = 24.8
    motorAnglePerRadian = 3.05
 

    noisyDistance = 0
end

function sysCall_sensing()
    
end



function getMaxMotorAngleFromTarget(posL, posR)

    -- How far are the left and right motors from their targets? Find the maximum
    maxAngle = 0
    if (speedBaseL > 0) then
        remaining = motorAngleTargetL - posL
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseL < 0) then
        remaining = posL - motorAngleTargetL
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseR > 0) then
        remaining = motorAngleTargetR - posR
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseR < 0) then
        remaining = posR - motorAngleTargetR
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end

    return maxAngle
end



function sysCall_actuation() 
    tt = sim.getSimulationTime()
    -- print("actuation hello", tt)
    

    -- Get and plot current angles of motor joints
    posL = sim.getJointPosition(leftMotor)
    posR = sim.getJointPosition(rightMotor)
    


    -- Start new step?
    if (stepCompletedFlag == true or stepCounter == 0) then
        stepCounter = stepCounter + 1
        stepCompletedFlag = false


        newStepType = stepList[stepCounter][1]

        if (newStepType == "repeat") then
            -- Loop back to the first step
            stepCounter = 1
            newStepType = stepList[stepCounter][1]
        end

        print("New step:", stepCounter, newStepType)

        if (newStepType == "forward") then
            -- Forward step: set new joint targets
            newStepAmount = stepList[stepCounter][2]
            motorAngleTargetL = posL + newStepAmount * motorAnglePerMetre
            motorAngleTargetR = posR + newStepAmount * motorAnglePerMetre
        elseif (newStepType == "turn") then
            -- Turn step: set new targets
            newStepAmount = stepList[stepCounter][2]
            motorAngleTargetL = posL - newStepAmount * motorAnglePerRadian
            motorAngleTargetR = posR + newStepAmount * motorAnglePerRadian
        elseif (newStepType == "stop") then
            print ("Stopping!")
        end
    end


    -- Handle current ongoing step
    stepType = stepList[stepCounter][1]

    if (stepType == "turn") then
        if (stepList[stepCounter][2] >= 0) then
            speedBaseL = -speedBase
            speedBaseR = speedBase
        else
            speedBaseL = speedBase
            speedBaseR = -speedBase
        end
        motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        -- Slow down when close
        if (motorAngleFromTarget < 3) then
            speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        if (motorAngleFromTarget == 0) then
            stepCompletedFlag = true
        end
    elseif (stepType == "forward") then
        speedBaseL = speedBase
        speedBaseR = speedBase
        motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        -- Slow down when close
        if (motorAngleFromTarget < 3) then
            speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        if (motorAngleFromTarget == 0) then
            stepCompletedFlag = true
        end
    elseif (stepType == "stop") then
        speedBaseL = 0
        speedBaseR = 0

        -- Check to see if the robot is stationary to within a small threshold
        linearVelocity,angularVelocity=sim.getVelocity(robotBase)
        vLin = math.sqrt(linearVelocity[1]^2 + linearVelocity[2]^2 + linearVelocity[3]^2)
        vAng = math.sqrt(angularVelocity[1]^2 + angularVelocity[2]^2 + angularVelocity[3]^2)
        --print ("stop", linearVelocity, vLin, vAng)

        if (vLin < 0.001 and vAng < 0.01) then
            stepCompletedFlag = true
        end
    end

    -- Set the motor velocities for the current step
    sim.setJointTargetVelocity(leftMotor,speedBaseL)
    sim.setJointTargetVelocity(rightMotor,speedBaseR)        

end

function sysCall_cleanup()
    --simUI.destroy(ui)
end
