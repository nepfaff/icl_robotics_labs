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

    -- Sequential state machine
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
    numberOfParticles = 100
    -- Initialise all particles to origin with uniform distribution
    -- We have certainty about starting position
    for i=1, numberOfParticles do
        xArray[i] = 0
        yArray[i] = 0
        -- ynew = 0 + (D+e)sin(0) will always give 0
        -- This is unrealistic but can be mitigated by initialising theta to a tiny
        -- bit of zero mean noise rather than to 0
        -- This would not be a problem in realistic scenarios as you could never be
        -- a 100% certain that you positioned your robot with an exact orientation
        thetaArray[i] = gaussian(0, 0.002)
        weightArray[i] = 1/numberOfParticles
        dummyArray[i] = sim.createDummy(0.05) -- Returns integer object handle

        -- Args: object handle, reference frame (-1 = absolute position), coordinates (x,y,z)
        sim.setObjectPosition(dummyArray[i], -1, {0,0,0})
        -- Args: object handle, reference frame (-1 = absolute position), euler angles (alpha, beta, gamma)
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,0})
    end

    -- Target positions for joints
    motorAngleTargetL = 0.0
    motorAngleTargetR = 0.0

     -- Motor angles in radians per unit (to calibrate)
    motorAnglePerMetre = 24.8
    motorAnglePerRadian = 3.05

    -- Zero mean Gaussian noise variance in meter/radians (to calibrate)
    -- Determined for a one meter distance
    straightLineXYVariance = 0.0005
    straightLineThetaVariance = 0.0005
    -- Zero mean Gaussian noise variance in radians (to calibrate)
    -- Determined for a one radian rotation
    rotationThetaVariance = 0.003
end

function sysCall_sensing()
    
end


-- Performs a particle motion prediction update for straight line motion
function updateParticlesAfterStraightLineMotion(metersMovedSinceLastUpdate)
    for i=1, numberOfParticles do
        -- Scale variances appropriately (variance is additive and determined for one meter)
        local distanceNoise = gaussian(0, straightLineXYVariance * metersMovedSinceLastUpdate)
        local rotationNoise = gaussian(0, straightLineThetaVariance * metersMovedSinceLastUpdate)

        local noisyDistance = metersMovedSinceLastUpdate + distanceNoise
        local noisyDistanceX = noisyDistance * math.cos(thetaArray[i])
        local noisyDistanceY = noisyDistance * math.sin(thetaArray[i])

        xArray[i] = xArray[i] + noisyDistanceX
        yArray[i] = yArray[i] + noisyDistanceY
        thetaArray[i] = thetaArray[i] + rotationNoise

        -- Args: object handle, reference frame (-1 = absolute position), coordinates (x,y,z)
        sim.setObjectPosition(dummyArray[i], -1, {xArray[i],yArray[i],0})
        -- Args: object handle, reference frame (-1 = absolute position), euler angles (alpha, beta, gamma)
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,thetaArray[i]})
    end
end


-- Performs a particle motion prediction update for pure rotation (rotation on the spot)
function updateParticlesAfterPureRotation(radiansRotatedSinceLastUpdate)
    for i=1, numberOfParticles do
        -- Scale variance appropriately (variance is additive and determined for one radian)
        local rotationNoise = gaussian(0, rotationThetaVariance * math.abs(radiansRotatedSinceLastUpdate))

        local noisyRoationRadians = radiansRotatedSinceLastUpdate + rotationNoise
        thetaArray[i] = thetaArray[i] + noisyRoationRadians

        -- Args: object handle, reference frame (-1 = absolute position), euler angles (alpha, beta, gamma)
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,thetaArray[i]})
    end
end


-- How far are the left and right motors from their targets? Find the maximum
function getMaxMotorAngleFromTarget(posL, posR)
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
        -- Set wheel speed based on turn direction
        if (stepList[stepCounter][2] >= 0) then
            -- Left turn
            speedBaseL = -speedBase
            speedBaseR = speedBase
        else
            -- Right turn
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
        -- Determine if we have reached the current step's goal
        if (motorAngleFromTarget == 0) then
            stepCompletedFlag = true

            -- Update particles
            rotationAmountRadians = stepList[stepCounter][2]
            updateParticlesAfterPureRotation(rotationAmountRadians)
        end
    elseif (stepType == "forward") then
        -- Set wheel speed
        speedBaseL = speedBase
        speedBaseR = speedBase

        motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        -- Slow down when close
        if (motorAngleFromTarget < 3) then
            speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        -- Determine if we have reached the current step's goal
        if (motorAngleFromTarget == 0) then
            stepCompletedFlag = true

            -- Update particles
            forwardAmountMeters = stepList[stepCounter][2]
            updateParticlesAfterStraightLineMotion(forwardAmountMeters)
        end
    elseif (stepType == "stop") then
        -- Set speed to zero
        speedBaseL = 0
        speedBaseR = 0

        -- Check to see if the robot is stationary to within a small threshold
        linearVelocity, angularVelocity = sim.getVelocity(robotBase)
        vLin = math.sqrt(linearVelocity[1]^2 + linearVelocity[2]^2 + linearVelocity[3]^2)
        vAng = math.sqrt(angularVelocity[1]^2 + angularVelocity[2]^2 + angularVelocity[3]^2)
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
