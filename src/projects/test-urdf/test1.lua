
local ffi = require("ffi")
uv = require("uv")
gnewt  = require( "ffi/newtoncapi" )
local timer = require('timer') 

------------------------------------------------------------------------------------------------------------

local simApp = {}
local identM = 	ffi.new("double[16]", { 
    [0]=1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0
})

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


function simApp:MakeChassisShape( collisionMask )

    local workd = gnewt.GetWorld()

    --local shape = dNewtonCollisionCompound(workd, 0);

    -- dMatrix offset (dGetIdentityMatrix());
    -- offset.m_posit.m_y = 0.7f;

    -- dNewtonCollisionConvexHull convex0 (workd, 8, &VehicleHullShape0[0][0], 3 * sizeof (dFloat), 0.001f, collisionMask);
    -- dNewtonCollisionConvexHull convex1 (workd, 8, &VehicleHullShape1[0][0], 3 * sizeof (dFloat), 0.001f, collisionMask);
    -- convex1.SetMatrix (&offset[0][0]);

    -- shape.BeginAddRemoveCollision();
    -- shape.AddCollision (&convex0);
    -- shape.AddCollision (&convex1);
    -- shape.EndAddRemoveCollision();
    return shape
end

------------------------------------------------------------------------------------------------------------

function ApplyGravity (body, timestep, threadIndex)

	-- apply gravity force to the body
	local mass
	local Ixx
	local Iyy
	local Izz

	gnewt.NewtonBodyGetMass(body, mass, Ixx, Iyy, Izz);
	gravityForce = ffi.new("struct gravityForce[1]", {[0] = 0.0, -9.8 * mass, 0.0, 0.0})
	gnewt.NewtonBodySetForce(body, gravityForce)
end

------------------------------------------------------------------------------------------------------------
local idc = 2

function simApp:makeBall( x, y, z, r )

    local shapeId = idc 
    idc = idc + 1
	-- crate a collision sphere
    local offM = ffi.new("double[9]", {[0]=0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
    local coll = gnewt.NewtonCreateSphere( self.client, r, shapeId, offM)

	-- create a dynamic body with a sphere shape, and 
    local iM = ffi.new("double[16]", {
        [0]=1.0, 0.0, 0.0, 0.0, 
        0.0, 1.0, 0.0, 0.0, 
        0.0, 0.0, 1.0, 0.0,
        z, y, z, 1.0
    })
	local body = gnewt.NewtonCreateDynamicBody(self.client, coll, iM);

	-- set the force callback for applying the force and torque
	gnewt.NewtonBodySetForceAndTorqueCallback(body, ApplyGravity)

	-- set the mass for this body
	local mass = 1.0
	gnewt.NewtonBodySetMassProperties(body, mass, coll)

	-- set the linear damping to zero
	gnewt.NewtonBodySetLinearDamping (body, 0.0)

	-- do no forget to destroy the collision after you not longer need it
	gnewt.NewtonDestroyCollision(coll)
	return body
end

------------------------------------------------------------------------------------------------------------
-- simStartup - general initialisation

function simApp:Startup()

    self.timer = require('timer') 

    ------------------------------------------------------------------------------------------------------------
    -- Start the GUI Server -- need to kill it on exit too.
    if serverLaunched == nil then
        self.timer.sleep(1000)
        serverLaunched = true
    end

    ------------------------------------------------------------------------------------------------------------

    self.client = gnewt.NewtonCreate();
    self.timer.sleep(500)

    --local cmd = gnewt.b3InitResetSimulationCommand(self.client)
    --local status = gnewt.b3SubmitClientCommandAndWaitStatus(self.client, cmd);
    ------------------------------------------------------------------------------------------------------------
    -- Some timers for use later
    self.time_start = os.time()
    self.time_last = os.clock()

    ------------------------------------------------------------------------------------------------------------
    -- Now load in some models.
    world = {}

    -- Load the world urdf
    local offM = ffi.new("double[16]", {[0]=0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
    world.physId = 1
    world.phys = gnewt.NewtonCreateBox(self.client, 0.0, 0.0, 0.0, world.physId, offM)
    p("Adding world...", world.physId)   

    ------------------------------------------------------------------------------------------------------------
    -- Note: The visual shape doesnt seem to seperately instance for each object
    --       so each object shares the same visual shape because they are identical
    balls = {}
    balls.test1 = self:makeBall(1.0, 0.0, 1.5, 0.5)    
    p("Adding Ball...", balls.test1)
    balls.test2 = self:makeBall(0.0, 0.0, 1.0, 0.5)    
    p("Adding Ball...", balls.test2)
    balls.test3 = self:makeBall(-1.0, 0.0, 0.5, 0.5)    
    p("Adding Ball...", balls.test3)
    -- Set sim to unitialised.
    self.simInit = 0

    ------------------------------------------------------------------------------------------------------------
    -- Install timer update - this will call simUpdate often.
    self.sdltimer = self.timer.setInterval(5, self.Update, self)
end 

------------------------------------------------------------------------------------------------------------
-- This SIM INIT method isnt ideal, but it will do for now.

function simApp:Update( ) 

    -- On the first step do some initialisation - seems to be necessary here.
    if self.simInit == 0 and self.client ~= nil then
        self.simInit = 1
        gnewt.NewtonSetNumberOfSubsteps(self.client, 6)
    end

    -- Physics updates - this will go in a coroutine.. to make it nice to run
    local time_current = os.clock()
    local dtime = time_current - self.time_last
    self.time_last = time_current

    if self.client ~= nil then
        p("tstep:", dtime)
        -- Update the physics  -- Can use Async here, will look at later.
        local cmd = gnewt.NewtonUpdate(self.client, dtime)
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

------------------------------------------------------------------------------------------------------------
