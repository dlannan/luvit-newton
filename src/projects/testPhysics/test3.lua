
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

------------------------------------------------------------------------------------------------------------

BUOYANCY     = 1

local vis   = require("visualutils")
local phys  = require("physics")

------------------------------------------------------------------------------------------------------------

local simApp = {}
local identM = gutil.identityMatrix()

------------------------------------------------------------------------------------------------------------
-- Comment this out to disable visual rendering
local VISUAL    = 1

------------------------------------------------------------------------------------------------------------
-- Identity counter
idc = 2

------------------------------------------------------------------------------------------------------------

local Cred      = ffi.new( "double[4]", { [0]=1.0, 0.0, 0.0, 1.0 } )
local Cgreen    = ffi.new( "double[4]", { [0]=0.0, 1.0, 0.0, 1.0 } )
local Cblue     = ffi.new( "double[4]", { [0]=0.0, 0.0, 1.0, 1.0 } )
local Cyellow   = ffi.new( "double[4]", { [0]=1.0, 1.0, 0.0, 1.0 } ) 
local Ccyan     = ffi.new( "double[4]", { [0]=0.0, 1.0, 1.0, 1.0 } )

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
    gnewt.NewtonDestroyCollision(collision)

    return body
end

------------------------------------------------------------------------------------------------------------

function simApp:addBody( iM, mass, coll )

    local body = gnewt.NewtonCreateDynamicBody(self.client, coll, ffi.cast("const double *const", iM))
    -- set the force callback for applying the force and torque
    gnewt.NewtonBodySetForceAndTorqueCallback(body, ffi.cast("NewtonApplyForceAndTorque", ApplyGravity))
    -- set the mass for this body
    gnewt.NewtonBodySetMassProperties(body, mass, coll)

    local defMatId = gnewt.NewtonMaterialGetDefaultGroupID( self.client )
    gnewt.NewtonBodySetMaterialGroupID(body, defMatId)
    return body
end

------------------------------------------------------------------------------------------------------------

function simApp:addMotorToHull( hull, x, y, z, mass )

    local shapeId = idc 
    idc = idc + 1

    local coll = gnewt.NewtonCreateBox( self.client, 0.1, 1.0, 1.0, shapeId, nil )

    local pinDir0 = ffi.new("double[4]", {0.0, 1.0, 0.0, 0.0})
    local pinDir1= ffi.new("double[4]", {0.0, 1.0, 0.0, 0.0})
    local pivotPoint = ffi.new("double[4]", {x, y, z-2.5, 0.0})

    --local joint = gnewt.NewtonConstraintCreateCorkscrew( self.client, pivotPoint, pinDir0, body, hull.body )   
    --local joint = gnewt.NewtonConstraintCreateUpVector( self.client, pinDir0, body )

    -- -- gnewt.NewtonCorkscrewSetUserCallback(joint, ffi.cast("NewtonCorkscrewCallback", MotorJointReady))
    --gnewt.NewtonJointSetStiffness( joint, 0.99 )
    return coll, joint
end


------------------------------------------------------------------------------------------------------------

function simApp:makePlatform( x, y, z, sx, sy, sz, mass )

    local shapeId = idc 
    idc = idc + 1

    -- crate a collision plane
    local coll = gnewt.NewtonCreateBox( self.client, sx, sy, sz, shapeId, nil)
	return coll
end

------------------------------------------------------------------------------------------------------------

function simApp:makeBoat( colls, mass, x, y, z )

    idc = idc + 1
    local coll = gnewt.NewtonCreateCompoundCollision( self.client, idc )
    gnewt.NewtonCompoundCollisionBeginAddRemove(coll)

    for k,v in ipairs(colls) do
        node = gnewt.NewtonCompoundCollisionAddSubCollision(coll, v)
    end
    gnewt.NewtonCompoundCollisionEndAddRemove(coll)

    -- create a dynamic body with a sphere shape, and     
    local iM = gutil.identityMatrix()
    iM[0].m_posit.m_x = x
    iM[0].m_posit.m_y = y
    iM[0].m_posit.m_z = z

    local body = self:addBody( iM, mass, coll )
	-- do no forget to destroy the collision after you not longer need it
    gnewt.NewtonDestroyCollision(coll)
    for k,v in ipairs(colls) do
        gnewt.NewtonDestroyCollision(v)
    end

    local udata = ffi.new("userData[1]")
    udata[0] = { 3, mass, 0.5, 0.0 }
    table.insert(self.userDataList, udata)
    gnewt.NewtonBodySetUserData(body, udata)

    return body
end

------------------------------------------------------------------------------------------------------------

function simApp:makeBoatHull( x, y, z, r, length, mass )

    local shapeId = idc 
    idc = idc + 1

	-- crate a collision sphere
    local coll = gnewt.NewtonCreateCapsule( self.client, r, r, length, shapeId, nil)

	-- create a dynamic body with a sphere shape, and 
    --local iM = gutil.identityMatrix()
    -- local axis = ffi.new("dVector", {0.0, 1.0, 0.0, 0.0})
    -- local iM = vis:rotationMatrix( axis, math.pi * 0.5 )
    -- iM[0].m_posit.m_x = x
    -- iM[0].m_posit.m_y = y
    -- iM[0].m_posit.m_z = z

    -- local body = self:addBody( iM, mass, coll )
	-- do no forget to destroy the collision after you not longer need it
    -- gnewt.NewtonDestroyCollision(coll)

    -- local udata = ffi.new("userData[1]")
    -- udata[0] = { r, mass, length, 0.0 }
    -- table.insert(self.userDataList, udata)
    -- gnewt.NewtonBodySetUserData(body, udata)
	return coll
end

------------------------------------------------------------------------------------------------------------

function simApp:updateHull( x, y, z )

    gl.glPushMatrix()
    gl.glColor3f(1,0,0)

    gl.glTranslated( x, y, z )
    gl.glRotated( 90, 0.0, 1.0, 0.0 )
    vis:DrawCylinder( 8, 8, 4.0, 0.5 )
    --vis:DrawSphere(8, 8, udata[0].radius)
    gl.glPopMatrix()

    gl.glPushMatrix()
    gl.glColor3f(1,0.5,0)

    gl.glTranslated( x, y, z - 2.5 )
    vis:DrawCubiod( 0.1, 0.5, 0.5 )
    gl.glPopMatrix()
end

------------------------------------------------------------------------------------------------------------

function simApp:updatePlatform( x, y, z )

    gl.glPushMatrix()
    gl.glColor3f(0,0,1)

    gl.glTranslated(x, y, z)
    vis:DrawCubiod( 2.0, 0.2, 1.0 )
    gl.glPopMatrix()
end

------------------------------------------------------------------------------------------------------------

function simApp:updateWorld()

    gl.glPushMatrix()
    gl.glColor3f(0.0, 1.0, 0.0)

    gl.glTranslatef(0.0, 0.0, 0.0)

    vis:DrawCubiod(50, 0.1, 50)
    gl.glPopMatrix()
end

------------------------------------------------------------------------------------------------------------

function simApp:updateBoat( body )

    local mat = ffi.new("double[16]")
    gnewt.NewtonBodyGetMatrix (body , mat)

    gl.glPushMatrix()
    gl.glMultMatrixd(mat)
    self:updateHull( -2, 0.0, 0.0 )
    self:updateHull( 2, 0.0, 0.0 )
    self:updatePlatform( 0.0, 1.0, 0.0 )
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
    self.motor1 = 0
    self.motor2 = 0

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
    boat = { hull1={}, hull2={}, plat={} }
    boat.hull1.coll = self:makeBoatHull(-2, 1.5, 0.0, 0.5, 4.0, 4.0)  
    boat.hull1.mat = ffi.new("double[16]")
    boat.hull1.motor, boat.hull1.motorJ = self:addMotorToHull( boat.hull1, -2.0, 1.5, 0.0, 2.0 )
    p("Adding hull1...", boat.hull1)

    boat.hull2.coll = self:makeBoatHull(2.0, 1.5, 0.0, 0.5, 4.0, 4.0)  
    boat.hull2.mat = ffi.new("double[16]")
    boat.hull2.motor, boat.hull2.motorJ = self:addMotorToHull( boat.hull2, 2.0, 1.5, 0.0, 2.0 )
    p("Adding hull2...", boat.hull2)

    boat.plat.coll = self:makePlatform(0.0, 2.5, 0.0, 4.0, 0.1, 2.0, 10)  
    boat.plat.mat = ffi.new("double[16]")
    p("Adding platform...", boat.plat)

    local colls = { boat.hull1.coll, boat.hull2.coll, boat.plat.coll, boat.hull1.motor, boat.hull2.motor }
    boat.body = self:makeBoat( colls, 14.0, 0.0, 2.0, 0.0 )
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

    self.motor1 = 0
    self.motor2 = 0

    -- Do some input checks here for applying force
    if glfw.GetMouseButton( self.window, 0 ) == 1 then self.motor1 = 1 end
    if glfw.GetMouseButton( self.window, 1 ) == 1 then self.motor2 = 1 end

    -- local udata = ffi.cast("userData *", gnewt.NewtonBodyGetUserData(boat.hull1.body))
    -- udata.motoron = self.motor1
    -- gnewt.NewtonBodySetUserData(boat.hull1.body, udata)
    -- local udata = ffi.cast("userData *", gnewt.NewtonBodyGetUserData(boat.hull2.body))
    -- udata.motoron = self.motor2
    -- gnewt.NewtonBodySetUserData(boat.hull2.body, udata)

    gl.glClear(bit.bor(gl.GL_COLOR_BUFFER_BIT, gl.GL_DEPTH_BUFFER_BIT))

    -- Setup the view of the cube. 
    vis:Perspective( 60.0, 1.0, 0.5, 100.0 )   
    vis:Camera( -7.0, 5.0, -7.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 )
    
    self:updateWorld()

    -- Go through the objects and update their visual. 
    self:updateBoat(boat.body)

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
