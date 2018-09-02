local ffi = require("ffi")
uv = require("uv")
newton  = require( "ffi/newtoncapi" )
gnewt = newton[1]
gutil = newton[2]

------------------------------------------------------------------------------------------------------------

local m_waterToSolidVolumeRatio = 0.9
local m_plane = ffi.new("double[4]", { [0]=0.0, 1.0, 0.0, 1.5 })

------------------------------------------------------------------------------------------------------------

local pwr = 50.0

function motorOn( body, enabled )

    -- if enabled apply a force, otherwise nothing?
    if enabled == 1 then 
        local mat = ffi.new("double[16]")
        gnewt.NewtonBodyGetMatrix( body, mat )
        local force = ffi.new("double[3]", { [0]=mat[2] * pwr, mat[6] * pwr, mat[10] * pwr })
        p(force[0], force[1], force[2])
        gnewt.NewtonBodyAddForce( body, force )
    end
end

------------------------------------------------------------------------------------------------------------

function MotorJointReady( screw, slider )

    return 0
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
-- p(udata[0], udata[1], udata[2])

    -- Must be below the plane to apply buoyancy
    if pos[1] < udata[0].radius then

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

    -- Check motor forces
    motorOn(body, udata[0].motoron)
end

------------------------------------------------------------------------------------------------------------
