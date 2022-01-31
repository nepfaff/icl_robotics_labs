-- Generate a sample from a Gaussian distribution
function gaussian (mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random())) *
            math.cos(2 * math.pi * math.random()) + mean
end


-- Return robot to a location
function resetBase(handle, matrix)
    -- get all objects in the model
    allModelObjects = sim.getObjectsInTree(handle)

    -- Temporary forbid automatic thread interruptions
    sim.setThreadAutomaticSwitch(false)

    -- reset all objects in the model
    for i=1, #allModelObjects, 1 do
        sim.resetDynamicObject(allModelObjects[i])
    end

    -- Set the transformation matrix of the object tree
    sim.setObjectMatrix(handle,-1,matrix)

    sim.setThreadAutomaticSwitch(true)
end


function createRandomBumpyFloor()
    print ("Generating new random bumpy floor.")
    sim.setThreadAutomaticSwitch(false)

    -- Move existing bumpy floor if there already is one (similar effect to replacing with new one)
    if (heightField ~= nil) then
        -- Shift the bumpy floor in the x-direction
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
    for i=1, cellsPerSide*cellsPerSide, 1 do
        table.insert(cellHeights, gaussian(0, heightFieldNoise))
    end
    heightField=sim.createHeightfieldShape(0, 0, cellsPerSide, cellsPerSide, floorSize, cellHeights)

    -- Make the floor invisible
    -- Parameter id 10 refers to the object's visibility (https://www.coppeliarobotics.com/helpFiles/en/objectParameterIDs.htm)
    sim.setObjectInt32Param(heightField,10,0)

    sim.setThreadAutomaticSwitch(true)
end


-- This function is executed exactly once when the scene is initialised
function sysCall_init()
    tt = sim.getSimulationTime()
    print("Init hello", tt)

    -- Save object handles
    robotBase=sim.getObjectHandle(sim.handle_self) -- robot handle
    leftMotor=sim.getObjectHandle("leftMotor") -- Handle of the left motor
    rightMotor=sim.getObjectHandle("rightMotor") -- Handle of the right motor
    turretMotor=sim.getObjectHandle("turretMotor") -- Handle of the turret motor
    turretSensor=sim.getObjectHandle("turretSensor")

    -- We only update graphs every few steps because the simulation slows down otherwise
    UPDATE_GRAPHS_EVERY = 20
    graphSteps = 0

    -- Save graph handles
    motorGraph=sim.getObjectHandle("motorGraph")
    trajectoryGraph=sim.getObjectHandle("trajectoryGraph")

    -- Create bumpy floor for robot to drive on
    createRandomBumpyFloor()

    -- Save robot start position so we can return it there later
    robotStartMatrix=sim.getObjectMatrix(robotBase,-1)

    -- Usual rotation rate for wheels (radians per second)
    speedBase = 1

    -- Which step are we in?
    -- 0 is a dummy value which is immediately completed
    stepCounter = 0

    -- State machine (states are traversed in order)
    stepList = {}
    -- Add steps for a full square consisting of four 1m forward motions
    -- separated by 90 degree left turns
    stepList[1] = {"forward", 1.00}
    stepList[2] = {"stop"}
    stepList[3] = {"turn", math.rad(90)}
    stepList[4] = {"stop"}
    stepList[5] = {"forward", 1.00}
    stepList[6] = {"stop"}
    stepList[7] = {"turn", math.rad(90)}
    stepList[8] = {"stop"}
    stepList[9] = {"forward", 1.00}
    stepList[10] = {"stop"}
    stepList[11] = {"turn", math.rad(90)}
    stepList[12] = {"stop"}
    stepList[13] = {"forward", 1.00}
    stepList[14] = {"stop"}
    stepList[15] = {"turn", math.rad(90)}
    stepList[16] = {"stop"}
    stepList[17] = {"repeat"}

    -- Number of times that we executed the stepList
    executionCounter = 0
    maxExecutions = 10

    -- Save robot's final x, y coordinates for computing a covariance matrix
    final_x_positions = {}
    final_y_positions = {}

    -- Target positions for joints (radians)
    -- These are set when a new movement state is started
    motorAngleTargetL = 0.0
    motorAngleTargetR = 0.0

    -- To calibrate
    motorAnglePerMetre = 24.5
    motorAnglePerRadian = 13
end


function sysCall_sensing()
    
end


-- Returns true if we have reached the current target
function isCurrentTargetAchieved(posL, posR)
    -- Current target is stopping
    if (stepList[stepCounter][1] == "stop") then
        -- Calculate L2-norm of linear and angular velocity in all directions to obtain
        -- combined linear and angular velocity measures
        linearVelocity, angularVelocity=sim.getVelocity(robotBase)
        vLin = math.sqrt(linearVelocity[1]^2 + linearVelocity[2]^2 + linearVelocity[3]^2)
        vAng = math.sqrt(angularVelocity[1]^2 + angularVelocity[2]^2 + angularVelocity[3]^2)

        -- Check to see if the robot is stationary to within a small threshold
        if (vLin < 0.001 and vAng < 0.01) then
            return true
        else
            return false
        end
    end

    -- Start with returnVal = true and negate it if any parts of target are not reached
    returnVal = true
    if (speedBaseL > 0 and posL < motorAngleTargetL) then
        returnVal = false
    end
    if (speedBaseL < 0 and posL > motorAngleTargetL) then
        returnVal = false
    end
    if (speedBaseR > 0 and posR < motorAngleTargetR) then
        returnVal = false
    end
    if (speedBaseR < 0 and posR > motorAngleTargetR) then
        returnVal = false
    end

    return returnVal
end


function mean(t)
    local sum = 0
    for _, val in pairs(t) do
        sum = sum + val
    end

    return sum / #t
end


function var(t, m)
    m = m or mean(t)

    local sum = 0
    for _, val in pairs(t) do
        sum = sum + (val - m) ^ 2
    end

    return sum / #t
end


function cross_var(x, y, m_x, m_y)
    m_x = m_x or mean(x)
    m_y = m_y or mean(y)

    local sum = 0
    for i=1, #x do
        sum = sum + (x[i] - m_x) * (y[i] - m_y)
    end

    return sum / #x
end


function printCovarianceMatrix(x, y)
    -- Compute variances
    local x_mean = mean(x)
    local y_mean = mean(y)

    local x_var = var(x, x_mean)
    local y_var = var(y, y_mean)
    local cross_var = cross_var(x, y, x_mean, y_mean)

    -- Print covariance matrix
    print("Covariance matrix:")
    print("|" .. x_var .. " " .. cross_var .. "|")
    print("|" .. cross_var .. " " .. y_var .. "|")
end


function sysCall_actuation()
    tt = sim.getSimulationTime()
    -- print("actuation hello", tt)

    -- Get current motor joint angles
    posL = sim.getJointPosition(leftMotor)
    posR = sim.getJointPosition(rightMotor)

    -- Plot current motor joint angles
    if graphSteps % UPDATE_GRAPHS_EVERY == 0 then
        sim.setGraphUserData(motorGraph,"leftPos",posL)
        sim.setGraphUserData(motorGraph,"rightPos",posR)
        sim.handleGraph(sim.handle_all, tt+sim.getSimulationTimeStep())
    end
    graphSteps = graphSteps + 1

    -- If we have got to the target of this step: move to the next step
    if (stepCounter == 0 or isCurrentTargetAchieved(posL, posR)) then
        -- Increment state machine
        stepCounter = stepCounter + 1
        print("Starting step", stepCounter)

        -- Determine new state
        newStepType = stepList[stepCounter][1]
        newStepAmount = stepList[stepCounter][2]
        print("Step type", newStepType, "Step amount",  newStepAmount)

        -- Execute action of new state
        if (newStepType == "forward") then
            -- Set new wheel movement amount targets
            -- 'newStepAmount' is the amount in meters that we want to move forward
            -- (we want both wheel angles to increase by same amount)
            -- Goal pos = current pos + target distance in motor rotation angle
            motorAngleTargetL = posL + newStepAmount * motorAnglePerMetre
            motorAngleTargetR = posR + newStepAmount * motorAnglePerMetre

            -- Set both wheels to the same positive speed to move forwards
            speedBaseL = speedBase
            speedBaseR = speedBase
        elseif (newStepType == "turn") then
            -- Set new wheel movement amount targets
            -- 'newStepAmount' is the amount in radians that we want to turn right
            -- (right wheel turns forward, left wheel turns backwards to turn on the spot)
            -- Goal pos = current pos + target distance in motor rotation angle
            motorAngleTargetL = posL - newStepAmount * motorAnglePerRadian
            motorAngleTargetR = posR + newStepAmount * motorAnglePerRadian

            -- Right turn on the spot: Right wheel moves forward and left wheel moves backwards at same speed
            speedBaseL = -speedBase
            speedBaseR = speedBase
        elseif (newStepType == "stop") then
            -- Target is to stop the robot (zero speed)
            speedBaseL = 0
            speedBaseR = 0
        elseif (newStepType == "repeat" and executionCounter < maxExecutions) then
            -- Drop a 'dummy' as a marker of the robot's final position
            newDummy = sim.createDummy(0.05)
            linearPosition = sim.getObjectPosition(robotBase,-1)
            sim.setObjectPosition(newDummy, -1, {linearPosition[1],linearPosition[2],0.0})

            -- Teleport robot back to the origin
            -- Just for the purposes of our experiment where we want the robot
            -- to repeat the same motion multiple times
            resetBase(robotBase, robotStartMatrix)

            -- Regenerate new bumpy floor for next run
            createRandomBumpyFloor()

            -- Reset state machine and graph update interval counter
            stepCounter = 0
            graphSteps = 0

            executionCounter = executionCounter + 1

            -- Save final x, y coordinates
            pos = sim.getObjectPosition(newDummy, -1)
            final_x_positions[executionCounter] = pos[1]
            final_y_positions[executionCounter] = pos[2]
        elseif (newStepType == "repeat" and executionCounter == maxExecutions) then
            -- Print covariance matrix
            printCovarianceMatrix(final_x_positions, final_y_positions)
        else
            print("ERROR: Invalid step type")
        end
    end

    -- Set the motor velocities for the current step
    sim.setJointTargetVelocity(leftMotor, speedBaseL)
    sim.setJointTargetVelocity(rightMotor, speedBaseR)
end

function sysCall_cleanup()
    --simUI.destroy(ui)
end
