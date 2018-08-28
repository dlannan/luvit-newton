
local ffi = require("ffi")
uv = require("uv")
newton  = require( "ffi/newtoncapi" )
gnewt = newton[1]
gutil = newton[2]

------------------------------------------------------------------------------------------------------------

local timer = require("timer") 

gl    = require( "ffi/OpenGL" )
glu   = require( "ffi/glu" )

glfw = require 'ffi/glfw' ('glfw3')
GLFW = glfw.const

local vis   = require("visualutils")

------------------------------------------------------------------------------------------------------------

local simApp = {}
local identM = gutil.identityMatrix()

local m_waterToSolidVolumeRatio = 0.9
local m_plane = ffi.new("double[4]", { [0]=0.0, 1.0, 0.0, 1.5 })

------------------------------------------------------------------------------------------------------------
-- Comment this out to disable visual rendering
--local VISUAL    = 1

------------------------------------------------------------------------------------------------------------

-- local basicCarParameters = ffi.new("BasciCarParameters[1]", 
-- {
-- 	[0] = 2500.0,	-- MASS
-- 	100.0,	-- TIRE_MASS
-- 	25.0,	-- STEER_ANGLE
-- 	10000.0,	-- BRAKE_TORQUE
-- 	-0.6,	-- COM_Y_OFFSET
-- 	120.0,		-- TIRE_TOP_SPEED_KMH
-- 	400.0,	-- IDLE_TORQUE
-- 	500.0,		-- IDLE_TORQUE_RPM
-- 	500.0,	-- PEAK_TORQUE
-- 	3000.0,	-- PEAK_TORQUE_RPM
-- 	300.0,	-- PEAK_HP
-- 	4000.0,	-- PEAK_HP_RPM
-- 	50.0,		-- REDLINE_TORQUE
-- 	4500.0,	-- REDLINE_TORQUE_RPM
-- 	2.5,	-- GEAR_1
-- 	2.0,	-- GEAR_2
-- 	1.5,	-- GEAR_3
-- 	2.9,	-- REVERSE_GEAR
-- 	0.7,	-- SUSPENSION_LENGTH
-- 	700.0,	-- SUSPENSION_SPRING
-- 	40.0,	-- SUSPENSION_DAMPER
-- 	20.0,	-- LATERAL_STIFFNESS
-- 	10000.0,	-- LONGITUDINAL_STIFFNESS
-- 	1.5,	-- ALIGNING_MOMENT_TRAIL
-- 	gnewt.m_4WD,
--     identM
-- })

------------------------------------------------------------------------------------------------------------

local VehicleHullShape0 = ffi.new("double[8][3]",   
{
	{-2.3, 0.0, -0.9}, {-2.3, 0.0, 0.9}, {2.3, 0.0, -0.9}, {2.3, 0.0, 0.9},
	{-2.1, 0.7, -0.9}, {-2.1, 0.7, 0.9}, {2.1, 0.7, -0.9}, {2.1, 0.7, 0.9},
})

------------------------------------------------------------------------------------------------------------

local VehicleHullShape1 = ffi.new("double[8][3]",   
{
	{-1.5, 0.0, -0.9}, {-1.5, 0.0, 0.9}, {1.2, 0.0, -0.9}, {1.2, 0.0, 0.9},
	{-1.1, 0.7, -0.9}, {-1.1, 0.7, 0.9}, {0.8, 0.7, -0.9}, {0.8, 0.7, 0.9},
})

------------------------------------------------------------------------------------------------------------

local Cred = ffi.new( "double[4]",{ [0]=1.0, 0.0, 0.0, 1.0 } )
local Cgreen = ffi.new( "double[4]",{ [0]=0.0, 1.0, 0.0, 1.0 } )
local Cblue = ffi.new( "double[4]",{ [0]=0.0, 0.0, 1.0, 1.0 } )
local Cyellow = ffi.new( "double[4]",{ [0]=1.0, 1.0, 0.0, 1.0 } ) 
local Ccyan = ffi.new( "double[4]",{ [0]=0.0, 1.0, 1.0, 1.0 } )

------------------------------------------------------------------------------------------------------------

function simApp:makeGround( )

    local points = ffi.new("dFloat[4][3]", {
        {-100.0, 0.0,  100.0}, 
        { 100.0, 0.0,  100.0}, 
        { 100.0, 0.0, -100.0}, 
        {-100.0, 0.0, -100.0}, 
    })

    -- crate a collision tree
    local collision = gnewt.NewtonCreateTreeCollision (self.client, 0)

    -- start building the collision mesh
    gnewt.NewtonTreeCollisionBeginBuild (collision)

    -- add the face one at a time
    gnewt.NewtonTreeCollisionAddFace (collision, 4, ffi.cast("const double *const", points), 3 * ffi.sizeof("dFloat"), 0);

    -- finish building the collision
    gnewt.NewtonTreeCollisionEndBuild (collision, 1)

    -- create a body with a collision and locate at the identity matrix position 
    local body = gnewt.NewtonCreateDynamicBody(self.client, collision, ffi.cast("const double *const", identM))

    -- do no forget to destroy the collision after you not longer need it
    gnewt.NewtonDestroyCollision(collision);

    return body
end

------------------------------------------------------------------------------------------------------------

function ApplyGravity(body, timestep, threadIndex)

	-- apply gravity force to the body
    local mass = ffi.new("double[1]")
	local Ixx = ffi.new("double[1]")
	local Iyy = ffi.new("double[1]")
    local Izz = ffi.new("double[1]")

    gnewt.NewtonBodyGetMass(body, mass, Ixx, Iyy, Izz)

    local pos = ffi.new("double[4]")
    gnewt.NewtonBodyGetPosition( body, pos )

    local udata = ffi.cast("userData *", gnewt.NewtonBodyGetUserData(body))
-- p(pos[0], pos[1], pos[2])

    -- Must be below the plane to apply buoyancy
    if pos[1] < 0.0 then
    local cog = ffi.new("dVector[1]")
    cog[0] = { [0]=0.0, 0.0, 0.0, 0.0 }
    local accelPerUnitMass = ffi.new("dVector[1]")
    local torquePerUnitMass = ffi.new("dVector[1]")
    local matrix = ffi.new("dMatrix[1]")
    local gravity = ffi.new("double[4]", { [0]=0.0, -9.8, 0.0, 0.0 })

    gnewt.NewtonBodyGetMatrix (body, ffi.cast("dFloat *", matrix))
    gnewt.NewtonBodyGetCentreOfMass(body, ffi.cast("dFloat *", cog))
    cog = dMatrixTransformVector(matrix, cog)
    --p("COG:", cog[0].m_x, cog[0].m_y, cog[0].m_z, cog[0].m_w)

    local collision = gnewt.NewtonBodyGetCollision(body)
    local shapeVolume = gnewt.NewtonConvexCollisionCalculateVolume(collision)
    local fluidDensity = 1.0 / (m_waterToSolidVolumeRatio * shapeVolume)
    local viscosity = 0.995

    gnewt.NewtonConvexCollisionCalculateBuoyancyAcceleration (collision, 
                ffi.cast("dFloat *", matrix), 
                ffi.cast("dFloat *", cog), gravity, m_plane, fluidDensity, viscosity, 
                ffi.cast("dFloat *", accelPerUnitMass), 
                ffi.cast("dFloat *", torquePerUnitMass))

    local force = dVectorScale(accelPerUnitMass, mass[0])
    local torque = dVectorScale(torquePerUnitMass, mass[0])

    local omega = ffi.new("dVector[1]")
    omega[0] = { [0]=0.0, 0.0, 0.0, 0.0 }
    gnewt.NewtonBodyGetOmega(body, ffi.cast("dFloat *", omega))
    omega = dVectorScale (omega, viscosity)
    gnewt.NewtonBodySetOmega(body, ffi.cast("dFloat *", omega))

    gnewt.NewtonBodyAddForce (body, ffi.cast("dFloat *", force))
    gnewt.NewtonBodyAddTorque (body, ffi.cast("dFloat *", torque))
    
    else

	local gravityForce = ffi.new("double[4]", {[0]=0.0, -9.8 * mass[0], 0.0, 0.0})
    gnewt.NewtonBodyAddForce(body, gravityForce)
    end
end

------------------------------------------------------------------------------------------------------------
idc = 2

function simApp:makeBall( x, y, z, r, mass )

    local shapeId = idc 
    idc = idc + 1
	-- crate a collision sphere
    local offM = gutil.identityMatrix()
    local coll = gnewt.NewtonCreateSphere( self.client, r, shapeId, ffi.cast("const double *const", offM))
    
	-- create a dynamic body with a sphere shape, and 
    local iM = gutil.identityMatrix()
    iM[0].m_posit.m_x = x
    iM[0].m_posit.m_y = y
    iM[0].m_posit.m_z = z
	local body = gnewt.NewtonCreateDynamicBody(self.client, coll, ffi.cast("const double *const", iM))

	-- set the force callback for applying the force and torque
	gnewt.NewtonBodySetForceAndTorqueCallback(body, ffi.cast("NewtonApplyForceAndTorque", ApplyGravity))

	-- set the mass for this body
	gnewt.NewtonBodySetMassProperties(body, mass, coll)

	-- set the linear damping to zero
    -- gnewt.NewtonBodySetLinearDamping (body, 0.1)
    local defMatId = gnewt.NewtonMaterialGetDefaultGroupID( self.client)
    gnewt.NewtonBodySetMaterialGroupID(body, defMatId)

	-- do no forget to destroy the collision after you not longer need it
    gnewt.NewtonDestroyCollision(coll)

    local udata = ffi.new("userData[1]")
    udata[0] = { r, mass, 0.0, 0.0 }
    table.insert(self.userDataList, udata)

    gnewt.NewtonBodySetUserData(body, udata)
       
	return body
end

------------------------------------------------------------------------------------------------------------

function simApp:updateBall( ball )

    gl.glPushMatrix()
    gl.glColor3f(1,0,0)

    local pos = ffi.cast("dMatrix *", ball.mat).m_posit
    gl.glTranslatef(pos.m_x, pos.m_y, pos.m_z)

    local udata = ffi.cast("userData *", gnewt.NewtonBodyGetUserData(ball.body))
    vis:DrawSphere(8, 8, udata[0].radius)
    gl.glPopMatrix()
end

------------------------------------------------------------------------------------------------------------

function simApp:updateWorld()

    gl.glPushMatrix()
    gl.glColor3f(0.0, 1.0, 0.0)

    gl.glTranslatef(0.0, 0.0, 0.0)

    vis:DrawPlane(100, 5)
    gl.glPopMatrix()
end

------------------------------------------------------------------------------------------------------------

function windowClosing()

    p("Stopping timer...")
    simApp.timer.clearInterval( simApp.phystimer)
    simApp.timer.clearInterval( simApp.vistimer)
end

------------------------------------------------------------------------------------------------------------
-- simStartup - general initialisation

function simApp:Startup()

    self.timer = require('timer') 
    self.userDataList = {}

    if VISUAL then
        -- Initialize the library
        if glfw.Init() == 0 then
            return
        end

        -- Create a windowed mode window and its OpenGL context
        self.window = glfw.CreateWindow(640, 480, "Hello World")
        p(self.window)
        if self.window == 0 then
            glfw.Terminate()
            return
        end

        -- Make the window's context current
        glfw.MakeContextCurrent(self.window)
        glfw.SetWindowCloseCallback( self.window, windowClosing )

        gl.glClearColor( 0.55, 0.55, 0.55, 0 )
        gl.glShadeModel( gl.GL_SMOOTH ) 
        gl.glEnable( gl.GL_DEPTH_TEST )
    end 

    ------------------------------------------------------------------------------------------------------------
    -- Start the GUI Server -- need to kill it on exit too.
    if serverLaunched == nil then
        serverLaunched = true
    end

    ------------------------------------------------------------------------------------------------------------
    -- Now load in some models.
    world = {}

    ------------------------------------------------------------------------------------------------------------

    self.client = gnewt.NewtonCreate()
	-- for deterministic behavior call this function each time you change the world
    gnewt.NewtonInvalidateCache (self.client)
    
    -- create a static body to serve as the floor.
    --world.physId = 1
    --p("Adding world...", world.physId)   
    --world.phys = self:makeGround( world.physId )

    ------------------------------------------------------------------------------------------------------------
    -- Some timers for use later
    self.time_start = os.time()
    self.time_last = os.clock()

    ------------------------------------------------------------------------------------------------------------
    -- Note: The visual shape doesnt seem to seperately instance for each object
    --       so each object shares the same visual shape because they are identical
    balls = { test1={}, test2={}, test3={} }
    balls.test1.body = self:makeBall(15.0, 15.0, 15.0, 5.0, 10.0)  
    balls.test1.mat = ffi.new("double[16]")
    p("Adding Ball...", balls.test1)
    balls.test2.body = self:makeBall(10.0, 15.0, 14.0, 5.0, 10.0)    
    balls.test2.mat = ffi.new("double[16]")
    p("Adding Ball...", balls.test2)
    balls.test3.body = self:makeBall(-15.0, 15.0, 12.0, 5.0, 10.0)    
    balls.test3.mat = ffi.new("double[16]")
    p("Adding Ball...", balls.test3)
    -- Set sim to unitialised.
    self.simInit = 0

    ------------------------------------------------------------------------------------------------------------
    -- Install timer update - this will call simUpdate often.
    self.phystimer = self.timer.setInterval(5, self.Update, self)
    if VISUAL then 
        self.vistimer = self.timer.setInterval(15, self.Render, self)
    end
end 

------------------------------------------------------------------------------------------------------------

function simApp:Render()

    gl.glClear(bit.bor(gl.GL_COLOR_BUFFER_BIT, gl.GL_DEPTH_BUFFER_BIT))

    -- Setup the view of the cube. 
    vis:Perspective( 60.0, 1.0, 0.5, 100.0 )   
    vis:Camera( -20.0, 5.0, -20.0, 140.0, -10.0, 0.0 )
    
    self:updateWorld()

    -- Go through the objects and update their visual. 
    gnewt.NewtonBodyGetMatrix (balls.test1.body , balls.test1.mat)
    self:updateBall( balls.test1 )
    local pos = ffi.cast("dMatrix *", balls.test1.mat).m_posit

    gnewt.NewtonBodyGetMatrix (balls.test2.body , balls.test2.mat)
    self:updateBall( balls.test2 )
    local pos = ffi.cast("dMatrix *", balls.test2.mat).m_posit

    gnewt.NewtonBodyGetMatrix (balls.test3.body , balls.test3.mat)
    self:updateBall( balls.test3 )
    local pos = ffi.cast("dMatrix *", balls.test3.mat).m_posit

    -- Swap front and back buffers
    glfw.SwapBuffers(self.window)

    -- Poll for and process events
    glfw.PollEvents()
end

------------------------------------------------------------------------------------------------------------
-- This SIM INIT method isnt ideal, but it will do for now.

function simApp:Update( ) 

    -- On the first step do some initialisation - seems to be necessary here.
    if self.simInit == 0 and self.client ~= nil then
        self.simInit = 1
        --gnewt.NewtonSetNumberOfSubsteps(self.client, 6)
    end

    -- Physics updates - this will go in a coroutine.. to make it nice to run
    local time_current = os.clock()
    local dtime = time_current - self.time_last
    self.time_last = time_current

    if self.client ~= nil then
        --p("tstep:", dtime)
        -- Update the physics  -- Can use Async here, will look at later.
        gnewt.NewtonUpdate(self.client, dtime)
    end


end

------------------------------------------------------------------------------------------------------------

coroutine.wrap( function() 
    simApp:Startup()
end)()

uv.run()

------------------------------------------------------------------------------------------------------------

p("Closing...")

------------------------------------------------------------------------------------------------------------

gnewt.NewtonDestroy(simApp.client)
glfw.Terminate()

------------------------------------------------------------------------------------------------------------
