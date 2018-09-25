
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
-- Set this to 1 if you want buoyancy on the XZ plane.
BUOYANCY = 0

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

function simApp:makeWall( x, y, z, length, height )

    local ystart = y
    local bodies = {}

    idc = idc + 1
    local coll = gnewt.NewtonCreateBox( self.client, 0.5, 0.25, 0.25, idc, nil )

    --for h=1, height * 2, 1 do
    local h = 1
        local xstart = x - length * 0.5 + ( h % 2 ) * 0.5

        for l=1, length, 1 do

            -- create a dynamic body with a sphere shape, and     
            local iM = gutil.identityMatrix()
            iM[0].m_posit.m_x = 0.0 --xstart + (l-1)
            iM[0].m_posit.m_y = 1.0 --ystart + (h-1) * 0.5 + 0.25
            iM[0].m_posit.m_z = z
            local body = self:addBody( iM, 0.5, coll )
            bodies[idc] = body
        end
        --end
    -- do no forget to destroy the collision after you not longer need it
    gnewt.NewtonDestroyCollision(coll)
    return bodies
end

------------------------------------------------------------------------------------------------------------

function simApp:shootBall( x, y, z, r, mass, speed )

    local shapeId = idc 
    idc = idc + 1

	-- crate a collision sphere
    local coll = gnewt.NewtonCreateCapsule( self.client, r, r, length, shapeId, nil)

	return coll
end

------------------------------------------------------------------------------------------------------------

function simApp:renderWall( wall )

    for k,v in ipairs(wall) do

        local mat = ffi.new("double[16]")
        gnewt.NewtonBodyGetMatrix (v , mat)

        gl.glPushMatrix()
        gl.glColor3f(1,0.5,0)

        gl.glTranslated( mat[12], mat[13], mat[14] )
        vis:DrawCubiod( 0.25, 0.25, 0.5 )
        gl.glPopMatrix()
    end
end

------------------------------------------------------------------------------------------------------------

function simApp:renderBall( ball )

    gl.glPushMatrix()
    gl.glColor3f(0,0,1)

    gl.glTranslated(x, y, z)
    vis:DrawSphere( 5, 5, 0.3 )
    gl.glPopMatrix()
end

------------------------------------------------------------------------------------------------------------

function simApp:renderWorld()

    gl.glPushMatrix()
    gl.glColor3f(0.0, 1.0, 0.0)

    gl.glTranslatef(0.0, 0.0, 0.0)

    vis:DrawCubiod(50, 0.1, 50)
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

    if VISUAL then
        -- Initialize the library
        if glfw.Init() == 0 then
            return
        end

        -- Create a windowed mode window and its OpenGL context
        self.window = glfw.CreateWindow(640, 480, "Sample01 - brickwall")
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
    world.physId = 1
    --p("Adding world...", world.physId)   
    world.phys = self:makeGround( world.physId )

    ------------------------------------------------------------------------------------------------------------
    -- Some timers for use later
    self.time_start = os.time()
    self.time_last = os.clock()

    ------------------------------------------------------------------------------------------------------------
    -- Make the wall
    self.wall = self:makeWall( 0, 0, -10, 1, 1 )
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
    vis:Camera( -7.0, 5.0, -7.0, 0, 2, -10, 0.0, 1.0, 0.0 )

    self:renderWorld()

    local time_current = os.clock()
    local dtime = time_current - self.time_last

    -- Go through the objects and update their visual. 
    if self.wall then self:renderWall(self.wall) end
    if self.ball then self:renderBall( self.ball ) end

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

    -- Do some input checks here for applying force
    if glfw.GetMouseButton( self.window, 0 ) == 1 then
        self.ball = self:shootBall()
    end

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