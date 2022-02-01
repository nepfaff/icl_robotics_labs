-- Generate a sample from a Gaussian distribution
function gaussian (mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random())) *
            math.cos(2 * math.pi * math.random()) + mean
end


-- Return robot to a location
function resetBase(handle, matrix)
    allModelObjects = sim.getObjectsInTree(handle) -- get all objects in the model
    sim.setThreadAutomaticSwitch(false)
    for i=1,#allModelObjects,1 do
        sim.resetDynamicObject(allModelObjects[i]) -- reset all objects in the model
    end
    sim.setObjectMatrix(handle,-1,matrix)
    sim.setThreadAutomaticSwitch(true)
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
    trajectoryGraph=sim.getObjectHandle("trajectoryGraph")
    sensingGraph=sim.getObjectHandle("sensingGraph")

    -- We only update graphs every few steps because the simulation slows down otherwise
    UPDATE_GRAPHS_EVERY = 20
    graphSteps = 0

    -- Create bumpy floor for robot to drive on
    createRandomBumpyFloor()

    -- Save robot start position so we can return it there later
    robotStartMatrix=sim.getObjectMatrix(robotBase,-1)
   
    -- Usual rotation rate for wheels (radians per second)
    speedBase = 5

    -- Proportional control gain
    proportionalControlGain = 5

    -- Desired distance to wall in m
    desiredDistanceToWall = 0.25

    -- Angle of turret motor to left in degrees
    turretAngleDegrees = 70
    sim.setJointTargetPosition(turretMotor, math.rad(turretAngleDegrees))
    
    -- Target positions for joints
    motorAngleTargetL = 0.0
    motorAngleTargetR = 0.0

    -- To calibrate
    motorAnglePerMetre = 24.5
    motorAnglePerRadian = 3.01
 
    sensorStandardDeviation = 0.03
    sensorVariance = sensorStandardDeviation^2

    noisyDistance = 0
end

function sysCall_sensing()
    
end


function sysCall_actuation() 
    tt = sim.getSimulationTime()

    -- Get and plot current angles of motor joints
    posL = sim.getJointPosition(leftMotor)
    posR = sim.getJointPosition(rightMotor)
    
    graphSteps = graphSteps + 1
    if graphSteps % UPDATE_GRAPHS_EVERY == 0 then
        sim.handleGraph(sim.handle_all, tt+sim.getSimulationTimeStep())
    end

    result,cleanDistance=sim.readProximitySensor(turretSensor)
    if (result>0) then
        noisyDistance= cleanDistance + gaussian(0.0, sensorVariance)

        -- Add the data to the graph:
        sim.setGraphUserData(sensingGraph,"ND",noisyDistance)
    end

    distanceErr = noisyDistance - desiredDistanceToWall
    -- print("Distance:", noisyDistance, "Error", distanceErr)

    -- Right turn on the spot: Right wheel moves forward and left wheel moves backwards at same speed
    speedBaseL = speedBase - 0.5 * proportionalControlGain * distanceErr
    speedBaseR = speedBase + 0.5 * proportionalControlGain * distanceErr

    -- Set the motor velocities for the current step
    sim.setJointTargetVelocity(leftMotor,speedBaseL)
    sim.setJointTargetVelocity(rightMotor,speedBaseR)
    
end

function sysCall_cleanup()
    --simUI.destroy(ui)
end 
