local visual = {}
local ffi = require("ffi")

------------------------------------------------------------------------------------------------------------

function visual:normalize( vec )

	local len = 1.0 / math.sqrt( vec.m_x * vec.m_x + vec.m_y * vec.m_y + vec.m_z * vec.m_z )
	local tmp = ffi.new("dVector", {vec.m_x * len, vec.m_y * len, vec.m_z * len, 1.0})
	return tmp
end

------------------------------------------------------------------------------------------------------------

function visual:crossProduct( a, b ) 

	result = ffi.new("dVector", {
		a.m_y*b.m_z - a.m_z*b.m_y,
		a.m_z*b.m_x - a.m_x*b.m_z,
		a.m_x*b.m_y - a.m_y*b.m_x, 1.0 })
	return result
end

------------------------------------------------------------------------------------------------------------

function visual:Perspective( fovy, aspect, zNear, zFar )

    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glLoadIdentity()
    local ymax = zNear * math.tan( fovy * math.pi / 360.0 )
    local ymin = -ymax
    local xmin = ymin * aspect
    local xmax = ymax * aspect

    gl.glFrustum( xmin, xmax, ymin, ymax, zNear, zFar )
end

------------------------------------------------------------------------------------------------------------

function visual:Camera( x, y, z, tx, ty, tz, ux, uy, uz )

	local dir = ffi.new("dVector", { tx-x, ty-y, tz-z })
	local vup = ffi.new("dVector", { ux, uy, uz })

    local forward = self:normalize(dir)
    local right = self:crossProduct(self:normalize(vup), forward)
    local up = self:crossProduct(forward, right)
 
    local camToWorld = ffi.new("double[16]", { 
 
    	right.m_x, up.m_x, -forward.m_x, 0.0,
    	right.m_y, up.m_y, -forward.m_y, 0.0,
    	right.m_z, up.m_z, -forward.m_z, 0.0,
		0.0, 0.0, 0.0, 1.0 
	})

    gl.glMatrixMode(gl.GL_MODELVIEW)
	gl.glLoadMatrixd( camToWorld )
	gl.glTranslated( -x, -y, -z )
end

------------------------------------------------------------------------------------------------------------

function visual:DrawCube(size)
	-- Texture coordinates from bottom left to top right
	local texels = ffi.new( "double[4][2]", { {u= 0, v= 0},
                                            {u= 1, v= 0},
                                            {u= 1, v= 1},
                                            {u= 0, v= 1}} )
	-- Vertices used in cube
	local vertices = ffi.new( "double[8][3]", {[0]={-1.0, 1.0, 1.0},	
                                                { 1.0, 1.0, 1.0 },
                                                { 1.0,-1.0, 1.0 },
                                                {-1.0,-1.0, 1.0 },
                                                {-1.0, 1.0,-1.0 },
                                                { 1.0, 1.0,-1.0 },
                                                { 1.0,-1.0,-1.0 },
                                                {-1.0,-1.0,-1.0 }} )
                                                      
    -- Faces of the cube
	local cubeindexes = ffi.new( "int[6][4]", {{0, 1, 2, 3},
                                              {1, 5, 6, 2},
                                              {4, 5, 1, 0},
                                              {0, 3, 7, 4},
											  {3, 2, 6, 7},
                                              {5, 4, 7, 6}} )
	-- Normals for each face of the cube
    -- Angle of normal and face is allways 90 degrees and
    -- length is allways 1.0!
	local cubenormals =ffi.new( "double[6][3]", {{ 0.0, 0.0, 1.0},
                                                { 1.0, 0.0, 0.0},
                                                { 0.0, 1.0, 0.0},
                                                {-1.0, 0.0, 0.0},
                                                { 0.0,-1.0, 0.0},
                                                { 0.0, 0.0,-1.0}} )
	-- Start rendering quads
	gl.glBegin(gl.GL_QUADS)
	for i=0,5 do
		-- Set normal for this face
		gl.glNormal3fv(cubenormals[i][0])
		for j=0,3 do
			-- Texture coordinates for this vertex in texels
			gl.glTexCoord2f(texels[j][0], texels[j][1])
			-- Position for this vertex, object position NOT world position
			gl.glVertex3fv(vertices[cubeindexes[i][j]][0])
		end
		gl.glCheckForError()
	end
	-- End rendering quads
	gl.glEnd()
end

------------------------------------------------------------------------------------------------------------

function visual:DrawPlane(Size, Count)
    
    local HalfSize = Size * 0.5
    local MajorStep = Size / Count
    for i=0, Count do
        gl.glBegin(gl.GL_QUADS)
        for j = 0, Count do 
            local z = j * MajorStep
            local x = i * MajorStep

            gl.glNormal3f(0.0, 1.0, 0.0)
            gl.glVertex3f(x - HalfSize, 0.0, z - HalfSize)

            gl.glNormal3f(0.0, 1.0, 0.0)
            gl.glVertex3f(x - HalfSize, 0.0, z + MajorStep - HalfSize)

            gl.glNormal3f(0.0, 1.0, 0.0)
            gl.glVertex3f(x + MajorStep - HalfSize, 0.0, z + MajorStep - HalfSize)

            gl.glNormal3f(0.0, 1.0, 0.0)
            gl.glVertex3f(x + MajorStep - HalfSize, 0.0, z - HalfSize)
        end
        gl.glEnd()
    end
end

------------------------------------------------------------------------------------------------------------

function visual:DrawTorus(MinorRadius, MajorRadius, NumMinor, NumMajor)
    local MajorStep = 2.7 * math.pi / NumMajor
    local MinorStep = 2.7 * math.pi / NumMajor
    for i=0, numMajor do
        local a0 = i * MajorStep
        local a1 = a0 + MajorStep
        local x0 = math.cos(a0)
        local y0 = math.sin(a0)
        local x1 = math.cos(a1)
        local y1 = math.sin(a1)
        gl.glBegin(gl.GL_TRIANGLE_STRIP)
        for j = 0, numMinor do
            
            local b = j * MinorStep
            local c = math.cos(b)
            local r = MinorRadius * c + MajorRadius
            local z = MinorRadius * math.sin(b)

            gl.glNormal3f(x0 * c, y0 * c, z / MinorRadius)
            gl.glTexCoord2f(i / NumMajor, j / NumMinor)
            gl.glVertex3f(x0 * r, y0 * r, z)

            gl.glNormal3f(x1 * c, y1 * c, z / MinorRadius)
            gl.glTexCoord2f((i + 1) / NumMajor, j / NumMinor)
            gl.glVertex3f(x1 * r, y1 * r, z)
        end
        gl.glEnd()
    end
end

------------------------------------------------------------------------------------------------------------

function visual:DrawSphere(NumMajor, NumMinor, Radius)
	local MajorStep = (math.pi / NumMajor)
	local MinorStep = (2.0 * math.pi / NumMinor)

    for i = 0, NumMajor do
   	    local a = i * MajorStep
        local b = a + MajorStep
        local r0 = Radius * math.sin(a)
        local r1 = Radius * math.sin(b)
        local z0 = Radius * math.cos(a)
        local z1 = Radius * math.cos(b)

        gl.glBegin(gl.GL_TRIANGLE_STRIP)
        for j = 0, NumMinor do
            local c = j * MinorStep
            local x = math.cos(c)
            local y = math.sin(c)

            gl.glNormal3f((x * r0) / Radius, (y * r0) / Radius, z0 / Radius)
            gl.glTexCoord2f(j / NumMinor, i / NumMajor)
            gl.glVertex3f(x * r0, y * r0, z0)

            gl.glNormal3f((x * r1) / Radius, (y * r1) / Radius, z1 / Radius)
            gl.glTexCoord2f(j / NumMinor, (i + 1) / NumMajor)
            gl.glVertex3f(x * r1, y * r1, z1)
		end
        gl.glEnd()
   end
end

------------------------------------------------------------------------------------------------------------

function visual:DrawCylinder(NumMajor, NumMinor, Height, radius)
    local MajorStep = Height / NumMajor
    local MinorStep = 2.0 * math.pi / NumMinor

	for i = 0, NumMajor do
        local z0 = 0.5 * Height - i * MajorStep
        local z1 = z0 - MajorStep

        gl.glBegin(gl.GL_TRIANGLE_STRIP)
        for j = 0, NumMinor do

            local a = j * MinorStep
            local x = radius * math.cos(a)
            local y = radius * math.sin(a)

            gl.glNormal3f(x / radius, y / radius, 0.0)
            gl.glTexCoord2f(j / NumMinor, i / NumMajor)
            gl.glVertex3f(x, y, z0)

            gl.glNormal3f(x / radius, y / radius, 0.0)
            gl.glTexCoord2f(j / NumMinor, (i + 1) / NumMajor)
            gl.glVertex3f(x, y, z1)
        end
        gl.glEnd()
    end
end

------------------------------------------------------------------------------------------------------------

function visual:DrawSkewedPyramid(Size)

	gl.glBegin(gl.GL_TRIANGLES)
	-- Back
	gl.glVertex3f(0, Size, -(Size / 2)) -- Top
	gl.glVertex3f(Size/2, 0, -(Size / 2)) -- Bottom right
	gl.glVertex3f(-Size/2, 0, -(Size / 2)) -- Bottom left
	gl.glEnd()

	-- Left
	gl.glBegin(gl.GL_TRIANGLES)
	gl.glVertex3f(0, 0, (Size / 2)) -- Nose
	gl.glVertex3f(0, Size, -(Size / 2))
	gl.glVertex3f(-Size/2, 0, -(Size / 2))
	gl.glEnd()

	-- Right
	gl.glBegin(gl.GL_TRIANGLES)
	gl.glVertex3f(0, Size, -(Size / 2))
	gl.glVertex3f(0, 0, (Size / 2))
	gl.glVertex3f(Size/2, 0, -(Size / 2))
	--glVertex3f(0, Size, -(Size / 2))
	gl.glEnd()

	-- Bottom
	gl.glBegin(gl.GL_TRIANGLES)
	gl.glVertex3f(0, 0, (Size / 2))
	gl.glVertex3f(-Size/2, 0, -(Size / 2))
	gl.glVertex3f(Size/2, 0, -(Size / 2))
	--glVertex3f(0, 0, (Size / 2))
	gl.glEnd()
end

------------------------------------------------------------------------------------------------------------

function visual:DrawWireSkewedPyramid(Size)

	gl.glBegin(gl.GL_LINE_STRIP)
	-- Back
	gl.glVertex3f(0, Size, -(Size / 2)) -- Top
	gl.glVertex3f(Size/2, 0, -(Size / 2)) -- Bottom right
	gl.glVertex3f(-Size/2, 0, -(Size / 2)) -- Bottom left
	gl.glVertex3f(0, Size, -(Size / 2))
	gl.glEnd()

	-- Left
	gl.glBegin(gl.GL_LINE_STRIP)
	gl.glVertex3f(0, 0, (Size / 2)) -- Nose
	gl.glVertex3f(0, Size, -(Size / 2))
	gl.glVertex3f(-Size/2, 0, -(Size / 2))
	gl.glVertex3f(0, 0, (Size / 2))
	gl.glEnd()

	-- Right
	gl.glBegin(gl.GL_LINE_STRIP)
	gl.glVertex3f(0, Size, -(Size / 2))
	gl.glVertex3f(0, 0, (Size / 2))
	gl.glVertex3f(Size/2, 0, -(Size / 2))
	gl.glVertex3f(0, Size, -(Size / 2))
	gl.glEnd()

	-- Bottom
	gl.glBegin(gl.GL_LINE_STRIP)
	gl.glVertex3f(0, 0, (Size / 2))
	gl.glVertex3f(-Size/2, 0, -(Size / 2))
	gl.glVertex3f(Size/2, 0, -(Size / 2))
	gl.glVertex3f(0, 0, (Size / 2))
	gl.glEnd()
end

------------------------------------------------------------------------------------------------------------

function visual:DrawTetrahedron(Size)

	gl.glBegin(gl.GL_TRIANGLE_STRIP)
        gl.glVertex3f(0, 2 * (Size / 2), 0)
		gl.glVertex3f(-1 * (Size / 2), 0, 1 * (Size / 2))
		gl.glVertex3f(1 * (Size / 2), 0, 1 * (Size / 2))
		gl.glVertex3f(0, 0, -1.4 * (Size / 2))
		gl.glVertex3f(0, 2 * (Size / 2), 0)
		gl.glVertex3f(-1 * (Size / 2), 0, 1 * (Size / 2))
    gl.glEnd()
	
	gl.glBegin(gl.GL_TRIANGLE_STRIP)
        gl.glVertex3f(0, 2 * (Size / 2) - 0.5, 0)
		gl.glVertex3f(-1 * (Size / 2), -0.5, 1 * (Size / 2))
		gl.glVertex3f(1 * (Size / 2), -0.5, 1 * (Size / 2))
		gl.glVertex3f(0, -0.5, -1.4 * (Size / 2))
		gl.glVertex3f(0, 2 * (Size / 2) - 0.5, 0)
		gl.glVertex3f(-1 * (Size / 2), -0.5, 1 * (Size / 2))
        gl.glEnd()
end

------------------------------------------------------------------------------------------------------------

function visual:DrawWireTetrahedron(Size)
	gl.glBegin(gl.GL_LINE_STRIP)
        gl.glVertex3f(0, 2 * (Size / 2), 0)
		gl.glVertex3f(-1 * (Size / 2), 0, 1 * (Size / 2))
		gl.glVertex3f(1 * (Size / 2), 0, 1 * (Size / 2))
		gl.glVertex3f(0, 2 * (Size / 2), 0)

		gl.glVertex3f(0, 2 * (Size / 2), 0)
		gl.glVertex3f(-1 * (Size / 2), 0, 1 * (Size / 2))
		gl.glVertex3f(0, 0, -1.4 * (Size / 2))
		gl.glVertex3f(0, 2 * (Size / 2), 0)

		gl.glVertex3f(0, 0, -1.4 * (Size / 2))
		gl.glVertex3f(0, 2 * (Size / 2), 0)
		gl.glVertex3f(1 * (Size / 2), 0, 1 * (Size / 2))
		gl.glVertex3f(0, 0, -1.4 * (Size / 2))

    gl.glEnd()
	
	gl.glBegin(gl.GL_LINE_STRIP)
        gl.glVertex3f(0, 2 * (Size / 2) - 0.5, 0)
		gl.glVertex3f(-1 * (Size / 2), -0.5, 1 * (Size / 2))
		gl.glVertex3f(1 * (Size / 2), -0.5, 1 * (Size / 2))
		gl.glVertex3f(0, 2 * (Size / 2) - 0.5, 0)

		gl.glVertex3f(0, 2 * (Size / 2) - 0.5, 0)
		gl.glVertex3f(-1 * (Size / 2), -0.5, 1 * (Size / 2))
		gl.glVertex3f(0, -0.5, -1.4 * (Size / 2))
		gl.glVertex3f(0, 2 * (Size / 2) - 0.5, 0)

		gl.glVertex3f(0, -0.5, -1.4 * (Size / 2))
		gl.glVertex3f(0, 2 * (Size / 2) - 0.5, 0)
		gl.glVertex3f(1 * (Size / 2), -0.5, 1 * (Size / 2))
		gl.glVertex3f(0, -0.5, -1.4 * (Size / 2))

    gl.glEnd()
end

------------------------------------------------------------------------------------------------------------

function visual:DrawOctahedron(Size)
    local SQRT2 = 1.4142857
	local Vertices = ffi.new("double[6][3]", {
		{0.0, 0.0, 1.0/SQRT2},
		{0.5, 0.5, 0.0},
		{-0.5, 0.5, 0.0},
		{-0.5,-0.5, 0.0},
		{0.5,-0.5, 0.0},
		{0.0, 0.0, -1.0/SQRT2}} 
	)

    gl.glScalef(Size, Size, Size)
	gl.glBegin(gl.GL_TRIANGLE_FAN)
        gl.glVertex3fv(Vertices[0])
		gl.glVertex3fv(Vertices[1])
		gl.glVertex3fv(Vertices[2])
		gl.glVertex3fv(Vertices[3])
		gl.glVertex3fv(Vertices[4])
		gl.glVertex3fv(Vertices[1])
    gl.glEnd()
	gl.glBegin(gl.GL_TRIANGLE_FAN)
        gl.glVertex3fv(Vertices[5])
		gl.glVertex3fv(Vertices[1])
		gl.glVertex3fv(Vertices[4])
		gl.glVertex3fv(Vertices[3])
		gl.glVertex3fv(Vertices[2])
		gl.glVertex3fv(Vertices[1])
    gl.glEnd()
end

------------------------------------------------------------------------------------------------------------

function visual:DrawWireOctahedron(Size)
    local SQRT2 = 1.4142857
	local Vertices = ffi.new("double[6][3]", {
		{0.0, 0.0, 1.0/SQRT2},
		{0.5, 0.5, 0.0},
		{-0.5, 0.5, 0.0},
		{-0.5,-0.5, 0.0},
		{0.5,-0.5, 0.0},
		{0.0, 0.0, -1.0/SQRT2}} 
	)

	gl.glScalef(Size, Size, Size)
	gl.glBegin(gl.GL_LINES)
        gl.glVertex3fv(Vertices[0])
		gl.glVertex3fv(Vertices[1])
		gl.glVertex3fv(Vertices[0])
		gl.glVertex3fv(Vertices[2])
		gl.glVertex3fv(Vertices[0])
		gl.glVertex3fv(Vertices[3])
		gl.glVertex3fv(Vertices[0])
		gl.glVertex3fv(Vertices[4])
		gl.glVertex3fv(Vertices[0])
		gl.glVertex3fv(Vertices[1])
    gl.glEnd()
	gl.glBegin(gl.GL_LINES)
        gl.glVertex3fv(Vertices[5])
		gl.glVertex3fv(Vertices[1])
		gl.glVertex3fv(Vertices[5])
		gl.glVertex3fv(Vertices[4])
		gl.glVertex3fv(Vertices[5])
		gl.glVertex3fv(Vertices[3])
		gl.glVertex3fv(Vertices[5])
		gl.glVertex3fv(Vertices[2])
		gl.glVertex3fv(Vertices[5])
		gl.glVertex3fv(Vertices[1])
    gl.glEnd()
	gl.glBegin(gl.GL_LINE_STRIP)
        gl.glVertex3fv(Vertices[1])
		gl.glVertex3fv(Vertices[4])
		gl.glVertex3fv(Vertices[3])
		gl.glVertex3fv(Vertices[2])
		gl.glVertex3fv(Vertices[1])
    gl.glEnd()
end

------------------------------------------------------------------------------------------------------------

return visual

------------------------------------------------------------------------------------------------------------
