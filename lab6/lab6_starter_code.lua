-- Generate a sample from a Gaussian distribution
function gaussian (mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random() + 0.00001)) *
            math.cos(2 * math.pi * math.random()) + mean
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
     
    -- Please use noisyDistance= cleanDistance + gaussian(0.0, sensorVariance) for all sonar sensor measurements
    sensorStandardDeviation = 0.1
    sensorVariance = sensorStandardDeviation^2
    noisyDistance = 0

    -- Create bumpy floor for robot to drive on
    createRandomBumpyFloor()
 
    -- Data structure for walls (your program can use this)
    walls = {}
    -- Fill it by parsing the scene in the GUI
    N_WALLS = get_walls()
    -- walls now is an array of arrays with the {Ax, Ay, Bx, By} wall coordinates



    -- Data structure for goals (your program can use this)
    goals = {}
    -- Fill it by parsing the scene in the GUI
    N_GOALS = get_goals()
    -- goals now is an array of arrays with the {Gx, Gy} goal coordinates

    for g=1,N_GOALS do
        print ("Goal" ..tostring(g).. " Gx " ..tostring(goals[g][1]).. " Gy " ..tostring(goals[g][2]))
    end

 

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
  
  
       
    -- Your code here!
    
    -- EXAMPLE: student thinks they have reached a goal
    -- reachedGoal(1, robotBase)

    
    
end

function sysCall_sensing()
    
end




function sysCall_actuation() 
    tt = sim.getSimulationTime()
    -- print("actuation hello", tt)
 

    result,cleanDistance=sim.readProximitySensor(turretSensor)
    if (result>0) then
        noisyDistance= cleanDistance + gaussian(0.0, sensorVariance)
        --print ("Depth sensor reading ", noisyDistance)
    end


    -- Your code here!

end

function sysCall_cleanup()
    for g=1,N_GOALS do
        sim.setShapeColor(goalHandles[g], nil, sim.colorcomponent_ambient_diffuse, {1, 0, 0})
    end
end 
