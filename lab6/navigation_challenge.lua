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


-- Move robot to a location (only for use in random setup, not from your code!)
function setRobotPose(handle, x, y, theta)
    allModelObjects = sim.getObjectsInTree(handle) -- get all objects in the model
    sim.setThreadAutomaticSwitch(false)
    for i=1,#allModelObjects,1 do
        sim.resetDynamicObject(allModelObjects[i]) -- reset all objects in the model
    end
    pos = sim.getObjectPosition(handle, -1)
    sim.setObjectPosition(handle, -1, {x, y, pos[3]})
    sim.setObjectOrientation(handle, -1, {0, 0, theta})
    sim.setThreadAutomaticSwitch(true)
end


function get_walls()
    -- Disable error reporting
    local savedState=sim.getInt32Param(sim.intparam_error_report_mode)
    sim.setInt32Param(sim.intparam_error_report_mode,0)
    local N = 1
    while true do
        local handle = sim.getObjectHandle("Wall"..tostring(N))
        if handle <= 0 then
            break
        end

        -- Read position and shape of wall
        -- Assume here that it is thin and oriented either along the x axis or y axis

        -- We can now get the propertries of these walls, e.g....
        local pos = sim.getObjectPosition(handle, -1)
        local res,minx = sim.getObjectFloatParameter(handle,15)
        local res,maxx = sim.getObjectFloatParameter(handle,18)
        local res,miny = sim.getObjectFloatParameter(handle,16)
        local res,maxy = sim.getObjectFloatParameter(handle,19)

        --print("Position of Wall " .. tostring(N) .. ": " .. tostring(pos[1]) .. "," .. tostring(pos[2]) .. "," .. tostring(pos[3]))
        --print("minmax", minx, maxx, miny, maxy)

        local Ax, Ay, Bx, By
        if (maxx - minx > maxy - miny) then
            print("Wall " ..tostring(N).. " along x axis")
            Ax = pos[1] + minx
            Ay = pos[2]
            Bx = pos[1] + maxx
            By = pos[2]
        else
            print("Wall " ..tostring(N).. " along y axis")
            Ax = pos[1]
            Ay = pos[2] + miny
            Bx = pos[1]
            By = pos[2] + maxy
        end
        print (Ax, Ay, Bx, By)

        walls[N] = {Ax, Ay, Bx, By}
        N = N + 1
    end
    -- enable error reporting
    sim.setInt32Param(sim.intparam_error_report_mode,savedState)

    return N - 1
end


function get_goals()
    -- Disable error reporting
    local savedState=sim.getInt32Param(sim.intparam_error_report_mode)
    sim.setInt32Param(sim.intparam_error_report_mode,0)
    local N = 1
    goalHandles = {}
    while true do
        local handle = sim.getObjectHandle("Goal"..tostring(N))
        if handle <= 0 then
            break
        end
        
        goalHandles[N] = handle

        -- Read position of goal
        local pos = sim.getObjectPosition(handle, -1)
    
        print("Position of Goal " .. tostring(N) .. ": " .. tostring(pos[1]) .. "," .. tostring(pos[2]) .. "," .. tostring(pos[3]))

        goals[N] = {pos[1], pos[2]}
        N = N + 1
    end
    -- enable error reporting
    sim.setInt32Param(sim.intparam_error_report_mode,savedState)

    return N - 1
end

-- Robot should call this function when it thinks it has reached goal N
-- Second argument is the robot's handle
function reachedGoal(N, handle)
    green = {0, 1, 0}
    yellow = {1, 1, 0}
    blue = {0, 0, 1}
    grey = {0.5, 0.5, 0.5}

    local pos = sim.getObjectPosition(handle, -1)
    local xerr = pos[1] - goals[N][1]
    local yerr = pos[2] - goals[N][2]
    local err = math.sqrt(xerr^2 + yerr^2)
    local localpts = 0
    local colour = grey
    if (err < 0.05) then
        localpts = 3
        colour = green
    elseif (err < 0.1) then
        localpts = 2
        colour = yellow
    elseif (err < 0.2) then
        localpts = 1
        colour = blue
    end

    -- Colour the goal
    --local goalHandle = sim.getObjectHandle("Goal" .. tostring(N))
    sim.setShapeColor(goalHandles[N], nil, sim.colorcomponent_ambient_diffuse, colour)

    -- if we're not at final goal (which is where we started)
    if (localpts > 0 and goalsReached[N] == false) then
        goalsReached[N] = true
        totalPoints = totalPoints + localpts
        print ("Reached Goal" ..tostring(N).. " with error " ..tostring(err).. ": Points: " ..tostring(localpts))
    end

    -- at final goal: have we reached all goals?
    if (N == startGoal and localpts > 0) then
        local allGoalsReached = true
        for i=1,N_GOALS do
            if (goalsReached[i] == false) then
                allGoalsReached = false
            end
        end
        -- Yes... all goals achieved so calculate time
        if (allGoalsReached == true) then
            tt = sim.getSimulationTime() 
            timeTaken = tt - startTime
            timePoints = 0
            if (timeTaken < 60) then
                timePoints = 5
            elseif (timeTaken < 90) then
                timePoints = 4
            elseif (timeTaken < 120) then
                timePoints = 3
            elseif (timeTaken < 180) then
                timePoints = 2
            elseif (timeTaken < 240) then
                timePoints = 1
            end
            totalPoints = totalPoints + timePoints
            print ("FINISH at time" ..tostring(timeTaken).. " with total points " ..tostring(totalPoints))

            sim.pauseSimulation()
        end
    end

end


-- This function is executed exactly once when the scene is initialised
function sysCall_init()
    startTime = sim.getSimulationTime()
    print("Start Time", startTime)

    robotBase=sim.getObjectHandle(sim.handle_self) -- robot handle
    leftMotor=sim.getObjectHandle("leftMotor") -- Handle of the left motor
    rightMotor=sim.getObjectHandle("rightMotor") -- Handle of the right motor
    turretMotor=sim.getObjectHandle("turretMotor") -- Handle of the turret motor
    turretSensor=sim.getObjectHandle("turretSensor")

    -- Create bumpy floor for robot to drive on
    createRandomBumpyFloor()

    -- Data structure for goals (your program can use this)
    goals = {}
    -- Fill it by parsing the scene in the GUI
    N_GOALS = get_goals()
    -- goals now is an array of arrays with the {Gx, Gy} goal coordinates

    -- Keep track of where we started
    startingGoal = -1

    -- for g=1, N_GOALS do
    --     print ("Goal" ..tostring(g).. " Gx " ..tostring(goals[g][1]).. " Gy " ..tostring(goals[g][2]))
    -- end

    -- Randomise robot start position to one of the goals with random orientation
    startGoal = math.random(N_GOALS)
    startx = goals[startGoal][1]
    starty = goals[startGoal][2]
    startOrientation = math.random() * 2 * math.pi
    setRobotPose(robotBase, startx, starty, startOrientation)

    -- These variables are for keeping score, and they will be changed by reachedGoal() --- don't change them directly!
    totalPoints = 0
    goalsReached = {}
    for i=1,N_GOALS do
        goalsReached[i] = false
    end

    -- Usual rotation rate for wheels (radians per second)
    speedBase = 10 -- Max competition speed is 10 radians/second
    speedBaseL = 0
    speedBaseR = 0

    -- Which step are we in?
    -- 0 is a dummy value which is immediately completed
    stepCounter = 6 -- Start with measurement step
    stepCompletedFlag = false

    -- Sequential state machine (executed for each waypoint)
    stepList = {}
    stepList[1] = {"read_waypoint"}
    stepList[2] = {"turn"}
    stepList[3] = {"stop"} -- TODO: Experiment with removing stop steps
    stepList[4] = {"forward"}
    stepList[5] = {"stop"}
    stepList[6] = {"measurement"}
    stepList[7] = {"repeat"}

    -- Waypoints
    N_WAYPOINTS = 44
    startingWaypoint = -1 -- Waypoint at which we start and end (determined after first measurement)
    currentWaypoint = -1
    passedStartingWaypoint = false -- Used to differentiate start from end
    waypoints = {}
    waypoints[1] = {2,-2} -- Bottom right goal
    waypoints[2] = {2,-1.5}
    waypoints[3] = {2,-1}
    waypoints[4] = {2,-0.5}
    waypoints[5] = {2,0}
    waypoints[6] = {2,0.5}
    waypoints[7] = {2,1}
    waypoints[8] = {2,1.5}
    waypoints[9] = {2,2} -- Top right goal
    waypoints[10] = {1.5,1.5}
    waypoints[11] = {1,1}
    waypoints[12] = {0.5,0.5}
    waypoints[13] = {0,0} -- Centre goal
    waypoints[14] = {0.6,0.6}
    waypoints[15] = {1.25,1.25}
    waypoints[16] = {0.6,1.25}
    waypoints[17] = {0,1.25}
    waypoints[18] = {-0.5,1.25}
    waypoints[19] = {-1,1.25}
    waypoints[20] = {-1.5,1.25}
    waypoints[21] = {-2,1.25}
    waypoints[22] = {-2,1.65}
    waypoints[23] = {-2,2} -- Top left goal
    waypoints[24] = {-2,1.5}
    waypoints[25] = {-2,1}
    waypoints[26] = {-2,0.5}
    waypoints[27] = {-2,0}
    waypoints[28] = {-2,-0.5}
    waypoints[29] = {-2,-1} -- Bottom left goal
    waypoints[30] = {-1.5,-1}
    waypoints[31] = {-1,-1}
    waypoints[32] = {-1,-1.5}
    waypoints[33] = {-1,-1.75}
    waypoints[34] = {-1.5,-1.75}
    waypoints[35] = {-2,-1.75}
    waypoints[36] = {-2,-2.25}
    waypoints[37] = {-1.5,-2.25}
    waypoints[38] = {-1,-2.25}
    waypoints[39] = {-0.5,-2.25}
    waypoints[40] = {0,-2.25}
    waypoints[41] = {0.5,-2.25}
    waypoints[42] = {1,-2.25}
    waypoints[43] = {1.5,-2.25}
    waypoints[44] = {2,-2.25}

    -- Used to find waypoint associated with a goal
    goalToWaypointMapping = {}
    goalToWaypointMapping[1] = 13 -- (0, 0)
    goalToWaypointMapping[2] = 23 -- (-2, 2)
    goalToWaypointMapping[3] = 1 -- (2, -2)
    goalToWaypointMapping[4] = 9 -- (2, 2)
    goalToWaypointMapping[5] = 29 -- (-2, -1)

    -- Determines the difference between consequtive angles that the turret
    -- is rotated to during the measurement step (rotated -pi to pi)
    turretAngleDeltaRad = math.rad(15)
    turretAngleTarget = -(math.pi - 0.01)
    sim.setJointTargetPosition(turretMotor, turretAngleTarget)
    numberOfMeasurements = 0
    maxNumberOfMeasurements = 12

    -- Record a series of measurements to update particles together (only need to resample once)
    distanceMeasurements = {}
    turretAngleRads = {}

    -- Create and initialise arrays for particles, and display them with dummies
    xArray = {}
    yArray = {}
    thetaArray = {}
    weightArray = {}
    dummyArray = {}
    -- Both numberOfParticles and numberOfParticleSecondRound must be a multiple of N_GOALS!
    numberOfParticles = 1000
    numberOfParticleSecondRound = 100 -- Only need lots of particles to determine initial location
    numberOfDummes = numberOfParticleSecondRound

    initialiseParticles()
    updateParticleVisualisation()

    -- Target movements for reaching the current waypoint
    waypointRotationRadians = 0.0
    waypointDistanceMeter = 0.0

    -- Target positions for joints
    motorAngleTargetL = 0.0
    motorAngleTargetR = 0.0

    -- Data structure for walls
    walls = {}
    -- Fill it by parsing the scene in the GUI
    N_WALLS = get_walls() -- Modifis "walls" and returns number of walls
    -- walls now is an array of arrays with the {Ax, Ay, Bx, By} wall coordinates

    sensorStandardDeviation = 0.1
    sensorVariance = sensorStandardDeviation^2
    sensorNoiseConstant = 0.0001 -- Makes filter less aggressive in killing of particles

    noisyDistance = 0

     -- Motor angles in radians per unit (to calibrate)
    motorAnglePerMetre = 24.8
    motorAnglePerRadian = 3.05

    -- Zero mean Gaussian noise variance in meter/radians (to calibrate)
    -- Determined for a one meter distance
    straightLineXYVariance = 0.005
    straightLineThetaVariance = 0.005
    -- Zero mean Gaussian noise variance in radians (to calibrate)
    -- Determined for a one radian rotation
    rotationThetaVariance = 0.005
end

function sysCall_sensing()
end


-- Initialises equal number of particles at each of the 5 possible locations with random orientations.
function initialiseParticles()
    local numberOfParticlesPerLocation = numberOfParticles / N_GOALS
    local goalIdx = 1
    for i=1, numberOfParticles do
        if i > (goalIdx * numberOfParticlesPerLocation) then
            goalIdx = goalIdx + 1
        end

        xArray[i] = goals[goalIdx][1]
        yArray[i] = goals[goalIdx][2]
        thetaArray[i] = 2*math.pi * math.random() - math.pi -- Random in range [-pi, pi]
        weightArray[i] = 1.0/numberOfParticles
    end

    for i=1, numberOfDummes do
        dummyArray[i] = sim.createDummy(0.05) -- Returns integer object handle
    end
end


function updateParticleVisualisation()
    local j = 1
    for i=1, numberOfParticles do
        if i % (numberOfParticles / numberOfDummes) == 0 then
           -- Args: object handle, reference frame (-1 = absolute position), coordinates (x,y,z)
            sim.setObjectPosition(dummyArray[j], -1, {xArray[i],yArray[i],0.0})
            -- Args: object handle, reference frame (-1 = absolute position), euler angles (alpha, beta, gamma)
            sim.setObjectOrientation(dummyArray[j], -1, {0.0,0.0,thetaArray[i]})

            j = j + 1
        end
    end
end


-- Performs a particle motion prediction update for straight line motion.
-- Does not do anything if 'metersMovedSinceLastUpdate' is zero (robot did not do anything).
function updateParticlesAfterStraightLineMotion(metersMovedSinceLastUpdate)
    -- Don't increase uncertainty if we did not do a movement
    if (metersMovedSinceLastUpdate == 0) then
        return
    end

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
    end

    updateParticleVisualisation()

    print("Updated particles after straight line motion")
end


-- Performs a particle motion prediction update for pure rotation (rotation on the spot).
-- Does not do anything if 'radiansRotatedSinceLastUpdate' is zero (robot did not do anything).
function updateParticlesAfterPureRotation(radiansRotatedSinceLastUpdate)
    -- Don't increase uncertainty if we did not do a movement
    if (radiansRotatedSinceLastUpdate == 0) then
        return
    end

    for i=1, numberOfParticles do
        -- Scale variance appropriately (variance is additive and determined for one radian)
        local rotationNoise = gaussian(0, rotationThetaVariance * math.abs(radiansRotatedSinceLastUpdate))

        local noisyRoationRadians = radiansRotatedSinceLastUpdate + rotationNoise
        thetaArray[i] = thetaArray[i] + noisyRoationRadians
    end

    updateParticleVisualisation()

    print("Updated particles after pure rotation")
end


-- Euclidean distance between two points
function euclideanDistance(x1, y1, x2, y2)
    return math.sqrt((x1-x2)^2 + (y1-y2)^2)
end


-- Returns true if the point (x, y) lies on the line between two points (Ax, Ay) and (Bx, By).
function isPointOnLineBetweenTwoPoints(x, y, Ax, Ay, Bx, By)
    local distanceMargin = 0.01 -- Needed for floating point errors
    return math.abs(euclideanDistance(Ax, Ay, x, y) + euclideanDistance(x, y, Bx, By) - euclideanDistance(Ax, Ay, Bx, By)) < distanceMargin
end


-- (x, y, theta) is the hypthosesis of a single particle.
-- z is the sonar distance measurement.
function calculateLikelihood(x, y, theta, z)
    -- Compute expected depth measurement m, assuming robot pose (x, y, theta)
    local m = math.huge
    for _, wall in ipairs(walls) do
        Ax = wall[1]
        Ay = wall[2]
        Bx = wall[3]
        By = wall[4]

        local distanceToWall = ((By - Ay)*(Ax - x) - (Bx - Ax)*(Ay - y)) / ((By - Ay)*math.cos(theta) - (Bx - Ax)*math.sin(theta))

        if (distanceToWall < m and distanceToWall >= 0) then
            -- Check if the sonar should hit between the endpoint limits of the wall
            local intersectX = x + distanceToWall * math.cos(theta)
            local intersectY = y + distanceToWall * math.sin(theta)

            -- Only update m if the sonar would actually hit the wall
            if (isPointOnLineBetweenTwoPoints(intersectX, intersectY, Ax, Ay, Bx, By)) then
                m = distanceToWall
            end
        end
    end

    -- Compute likelihood based on difference between m and z
    local likelihood = math.exp(- (z - m)^2 / (2*sensorVariance)) + sensorNoiseConstant

    if (m == math.huge) then
        print("NO ACTUAL DISTANCE TO WALL FOUND: Assume likelihood is one")
        likelihood = 1.0
    end

    return likelihood
end


-- Returns the sum of all elements in an array.
function sum(array)
    local sum = 0
    for i=1, #array do
        sum = sum + array[i]
    end

    return sum
end


-- Perform particle filter normalisation step to ensure that weights add up to 1.
function normaliseParticleWeights()
    local weightSum = sum(weightArray)
    if weightSum == 0 then
        print("ERROR in normaliseParticleWeights: weightSum is zero")
    end

    for i=1, #weightArray do
        weightArray[i] = weightArray[i] / weightSum
    end
end


-- Perform particle resampling using biased roulette wheel method.
function resampleParticles()
    local cumulativeWeightArray = {weightArray[1]}
    for i=2, numberOfParticles do
        cumulativeWeightArray[i] = cumulativeWeightArray[i-1] + weightArray[i]
    end

    -- Number of particles of first round differs from remaining rounds
    if numberOfParticles ~= numberOfParticleSecondRound then
        numberOfParticles = numberOfParticleSecondRound

        weightArray = {}
    end

    local newXArray = {}
    local newYArray = {}
    local newThetaArray = {}
    for i=1, numberOfParticles do
        local r = math.random() -- Random number in range [0,1]
        for j=1, #cumulativeWeightArray do
            if (r <= cumulativeWeightArray[j]) then
                newXArray[i] = xArray[j]
                newYArray[i] = yArray[j]
                newThetaArray[i] = thetaArray[j]
                break
            end
        end
    end

    xArray = newXArray
    yArray = newYArray
    thetaArray = newThetaArray

    for i=1, numberOfParticles do
        weightArray[i] = 1 / numberOfParticles
    end
end


-- Perform particle measurement update
function updateParticlesAfterMeasurement(distanceMeasurements, turretAngleRads)
    -- Combined measurement update
    for i=1, numberOfParticles do
        for j=1, #distanceMeasurements do
             -- Account for turret rotation by adding turret angle to the particle's angle
            local likelihood = calculateLikelihood(xArray[i], yArray[i], thetaArray[i] + turretAngleRads[j], distanceMeasurements[j])
            weightArray[i] = weightArray[i] * likelihood
        end
    end

    normaliseParticleWeights()

    resampleParticles()

    updateParticleVisualisation()

    print("Updated particles after measurement update, normalisation, and resampling")
end


-- Takes an array of values and an array of corresponding weights.
-- Both arrays must be of the same length.
function weighted_sum(values, weights)
    local sum = 0.0
    for i=1, #values do
        sum = sum + weights[i] * values[i]
    end

    return sum
end


-- Transforms theta into the range -pi < deltaTheta <= pi by subtraction/addition of 2*pi
function normaliseThetaMinusPiToPlusPi(theta)
    local normalisedTheta = theta % (2.0*math.pi)
    if (normalisedTheta > math.pi) then
        normalisedTheta = normalisedTheta - 2.0*math.pi
    elseif (normalisedTheta < -math.pi) then
        normalisedTheta = normalisedTheta + 2.0*math.pi
    end

    return normalisedTheta
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


function getClosestGoalFromCoordinates(x, y)
    local closestGoal = -1
    local minDistance = math.huge

    for i=1, N_GOALS do
        local distanceToGoal = euclideanDistance(goals[i][1], goals[i][2], x, y)
        if distanceToGoal < minDistance then
            minDistance = distanceToGoal
            closestGoal = i
        end
    end

    return closestGoal
end


function pointEstimateX()
    return weighted_sum(xArray, weightArray)
end

function pointEstimateY()
    return weighted_sum(yArray, weightArray)
end

function pointEstimateTheta()
    return weighted_sum(thetaArray, weightArray)
end


function closeEnoughToGoal(goal)
    local goalPos = {};
    if goal == 1 then
        goalPos = {0, 0}
    elseif goal == 2 then
        goalPos = {-2, 2}
    elseif goal == 3 then
        goalPos = {2, -2}
    elseif goal == 4 then
        goalPos = {2, 2}
    else
        goalPos = {-2, -1}
    end

    -- Close enough if less than 5cm away (Use 4cm to be save)
    return euclideanDistance(goalPos[1], goalPos[2], pointEstimateX(), pointEstimateY()) < 0.04
end


function sysCall_actuation()
    tt = sim.getSimulationTime()

    -- Get current angles of motor joints
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

            if (passedStartingWaypoint) then
                if (currentWaypoint == goalToWaypointMapping[1]) then
                    if closeEnoughToGoal(1) then
                        print("Reached goal 1")
                        reachedGoal(1, robotBase)
                    else
                        currentWaypoint = currentWaypoint - 1
                    end
                elseif (currentWaypoint == goalToWaypointMapping[2]) then
                    if closeEnoughToGoal(2) then
                        print("Reached goal 2")
                        reachedGoal(2, robotBase)
                    else
                        currentWaypoint = currentWaypoint - 1
                    end
                elseif (currentWaypoint == goalToWaypointMapping[3]) then
                    if closeEnoughToGoal(3) then
                        print("Reached goal 3")
                        reachedGoal(3, robotBase)
                    else
                        currentWaypoint = currentWaypoint - 1
                    end
                elseif (currentWaypoint == goalToWaypointMapping[4]) then
                    if closeEnoughToGoal(4) then
                        print("Reached goal 4")
                        reachedGoal(4, robotBase)
                    else
                        currentWaypoint = currentWaypoint - 1
                    end
                elseif (currentWaypoint == goalToWaypointMapping[5]) then
                    if closeEnoughToGoal(5) then
                        print("Reached goal 5")
                        reachedGoal(5, robotBase)
                    else
                        currentWaypoint = currentWaypoint - 1
                    end
                elseif (currentWaypoint == N_WAYPOINTS) then
                    currentWaypoint = 0
                end
            end

            if (currentWaypoint == startingWaypoint) then
                if (passedStartingWaypoint) then
                    print("DESTINATION REACHED")
                    return
                else
                    passedStartingWaypoint = true
                end
            end

            currentWaypoint = currentWaypoint + 1
        end

        print("New step:", stepCounter, newStepType)

        if (newStepType == "read_waypoint") then
            print("Waypoint is: ", currentWaypoint)

            -- Read next waypoint
            local waypoint = waypoints[currentWaypoint]
            local goalX = waypoint[1]
            local goalY = waypoint[2]

            -- Set new movement targets to reach the new waypoint
            -- All calculations below use units meter and radian
            local currentX = pointEstimateX()
            local currentY = pointEstimateY()
            local currentTheta = pointEstimateTheta()

            local deltaX = goalX - currentX
            local deltaY = goalY - currentY
            -- Note that Lua math.atan implements atan2(dy,dx)
            local absoluteAngleToGoal = math.atan(deltaY, deltaX)
            local deltaTheta = absoluteAngleToGoal - currentTheta
            -- Make sure that -pi < deltaTheta <= pi for efficiency
            deltaTheta = normaliseThetaMinusPiToPlusPi(deltaTheta)

            waypointRotationRadians = deltaTheta
            waypointDistanceMeter = math.sqrt(deltaX^2 + deltaY^2)
        elseif (newStepType == "forward") then
            -- Forward step: set new joint targets
            motorAngleTargetL = posL + waypointDistanceMeter * motorAnglePerMetre
            motorAngleTargetR = posR + waypointDistanceMeter * motorAnglePerMetre
        elseif (newStepType == "turn") then
            -- Turn step: set new targets
            motorAngleTargetL = posL - waypointRotationRadians * motorAnglePerRadian
            motorAngleTargetR = posR + waypointRotationRadians * motorAnglePerRadian
        elseif (newStepType == "measurement") then
            print("Taking measurements")
        end
    end

    -- Handle current ongoing step
    stepType = stepList[stepCounter][1]

    if (stepType == "read_waypoint") then
        -- Directly move to next step
        stepCompletedFlag = true
    elseif (stepType == "turn") then
        -- Set wheel speed based on turn direction
        if (waypointRotationRadians >= 0) then
            -- Left turn
            speedBaseL = -speedBase
            speedBaseR = speedBase
        else
            -- Right turn
            speedBaseL = speedBase
            speedBaseR = -speedBase
        end

        local motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        -- Slow down when close
        if (motorAngleFromTarget < 3) then
            local speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        -- Determine if we have reached the current step's goal
        if (motorAngleFromTarget == 0.0) then
            stepCompletedFlag = true

            -- Update particles
            updateParticlesAfterPureRotation(waypointRotationRadians)
            waypointRotationRadians = 0.0
        end
    elseif (stepType == "forward") then
        -- Set wheel speed
        speedBaseL = speedBase
        speedBaseR = speedBase

        local motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        -- Slow down when close
        if (motorAngleFromTarget < 3) then
            local speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        -- Determine if we have reached the current step's goal
        if (motorAngleFromTarget == 0.0) then
            stepCompletedFlag = true

            -- Update particles
            updateParticlesAfterStraightLineMotion(waypointDistanceMeter)
            waypointDistanceMeter = 0.0
        end
    elseif (stepType == "stop") then
        -- Set speed to zero
        speedBaseL = 0
        speedBaseR = 0

        -- Check to see if the robot is stationary to within a small threshold
        local linearVelocity, angularVelocity = sim.getVelocity(robotBase)
        local vLin = math.sqrt(linearVelocity[1]^2 + linearVelocity[2]^2 + linearVelocity[3]^2)
        local vAng = math.sqrt(angularVelocity[1]^2 + angularVelocity[2]^2 + angularVelocity[3]^2)
        if (vLin < 0.001 and vAng < 0.01) then
            stepCompletedFlag = true
        end
    elseif (stepType == "measurement") then
        -- Set speed to zero
        speedBaseL = 0
        speedBaseR = 0

        --print("turret target", turretAngleTarget, "turret current", sim.getJointPosition(turretMotor))
        if numberOfMeasurements == maxNumberOfMeasurements then
            -- Finished all measurements

            -- Combined measurement update
            updateParticlesAfterMeasurement(distanceMeasurements, turretAngleRads)
            distanceMeasurements = {}
            turretAngleRads = {}

            -- Reset turret
            turretAngleTarget = -(math.pi - 0.01)
            sim.setObjectInt32Param(turretMotor, sim.jointintparam_ctrl_enabled, 1) -- Put in position control mode
            sim.setJointTargetPosition(turretMotor, turretAngleTarget)

            -- Check if need to set starting location
            if startingWaypoint == -1 then
                startingGoal = getClosestGoalFromCoordinates(pointEstimateX(), pointEstimateY())
                startingWaypoint = goalToWaypointMapping[startingGoal]
                currentWaypoint = startingWaypoint
                print("Starting waypoint is:", startingWaypoint)
            end

            numberOfMeasurements = 0
            stepCompletedFlag = true
        else
            -- Rotate turret
            --sim.setJointTargetPosition(turretMotor, turretAngleTarget)
            sim.setObjectInt32Param(turretMotor, sim.jointintparam_ctrl_enabled, 0) -- Put in velocity control mode
            sim.setJointTargetVelocity(turretMotor, 10)

            -- Take measurement
            local turretAngleCurrent = sim.getJointPosition(turretMotor)
            local result, cleanDistance = sim.readProximitySensor(turretSensor)
            if result > 0 then
                noisyDistance = cleanDistance + gaussian(0.0, sensorVariance)

                distanceMeasurements[#distanceMeasurements+1] = noisyDistance
                turretAngleRads[#turretAngleRads+1] = turretAngleCurrent
            end

            turretAngleTarget = turretAngleTarget + turretAngleDeltaRad

            numberOfMeasurements = numberOfMeasurements + 1
        end
    end

    -- Set the motor velocities for the current step
    sim.setJointTargetVelocity(leftMotor,speedBaseL)
    sim.setJointTargetVelocity(rightMotor,speedBaseR)
end


function sleep(seconds)
    local t0 = os.clock()
    while os.clock() - t0 <= seconds do end
end


function sysCall_cleanup()
    for g=1,N_GOALS do
        sim.setShapeColor(goalHandles[g], nil, sim.colorcomponent_ambient_diffuse, {1, 0, 0})
    end
end
