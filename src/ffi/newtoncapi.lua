--
-- Created by David Lannan - copyright 2013
-- Developed for the Byt3D project. byt3d.codeplex.com
-- User: dlannan
-- Date: 21/04/13
-- Time: 8:28 PM
--
local ffi = require( "ffi" )

local libs = ffi_newton_libs or {
    --Windows = { x86 = "bin/newton_d.dll", x64 = "bin/newton_d.dll" },
    Windows = { x86 = "bin/newton.dll", x64 = "bin/newton.dll" },
    OSX     = { x86 = "/usr/lib/newton.dylib", x64 = "/usr/lib/newton.dylib" },
    Linux   = { x86 = "newton", x64 = "newton", arm = "newton" },
}

local lib   = ffi_newton_libs or libs[ ffi.os ][ ffi.arch ]
local newton= ffi.load( lib )

ffi.cdef [[

	typedef long long dLong;		
    typedef double dFloat;
	typedef double dFloat64;
    
    enum Physics {
	 NEWTON_BROADPHASE_DEFAULT						=0,
	 NEWTON_BROADPHASE_PERSINTENT					=1
    };

    enum BodyType {
	 NEWTON_DYNAMIC_BODY							=0,
	 NEWTON_KINEMATIC_BODY							=1,
	 NEWTON_DYNAMIC_ASYMETRIC_BODY					=2,
//	 NEWTON_DEFORMABLE_BODY							=2
    };

    enum Serialize { 
	SERIALIZE_ID_SPHERE								=0,
	 SERIALIZE_ID_CAPSULE							=1,
	 SERIALIZE_ID_CYLINDER							=2,
	 SERIALIZE_ID_CHAMFERCYLINDER					=3,
	 SERIALIZE_ID_BOX								=4	,
	 SERIALIZE_ID_CONE								=5,
	 SERIALIZE_ID_CONVEXHULL						=	6,
	 SERIALIZE_ID_NULL								=7,
	 SERIALIZE_ID_COMPOUND							=8,
	 SERIALIZE_ID_TREE								=9,
	 SERIALIZE_ID_HEIGHTFIELD						=10,
	 SERIALIZE_ID_CLOTH_PATCH						=11,
	 SERIALIZE_ID_DEFORMABLE_SOLID					=12,
	 SERIALIZE_ID_USERMESH							=13,
	 SERIALIZE_ID_SCENE								=14,
     SERIALIZE_ID_FRACTURED_COMPOUND				=	15,
    };


	typedef struct NewtonMesh{} NewtonMesh;
	typedef struct NewtonBody{} NewtonBody;
	typedef struct NewtonWorld{} NewtonWorld;
	typedef struct NewtonJoint{} NewtonJoint;
	typedef struct NewtonMaterial{} NewtonMaterial;
	typedef struct NewtonCollision{} NewtonCollision;
	typedef struct NewtonInverseDynamics{} NewtonInverseDynamics;
	typedef struct NewtonDeformableMeshSegment{} NewtonDeformableMeshSegment;
	typedef struct NewtonInverseDynamicsEffector {} NewtonInverseDynamicsEffector;
	typedef struct NewtonFracturedCompoundMeshPart{} NewtonFracturedCompoundMeshPart;

	typedef struct NewtonCollisionMaterial
	{
		void* m_userData;
		int m_userId;
		int m_userFlags;
		dFloat m_userParam[4];
	} NewtonCollisionMaterial;

	typedef struct NewtonBoxParam
	{
		dFloat m_x;
		dFloat m_y;
		dFloat m_z;
	} NewtonBoxParam;

	typedef struct NewtonSphereParam
	{
		dFloat m_radio;
	} NewtonSphereParam;


	typedef struct NewtonCapsuleParam
	{
		dFloat m_radio0;
		dFloat m_radio1;
		dFloat m_height;
	} NewtonCapsuleParam;

	typedef struct NewtonCylinderParam
	{
		dFloat m_radio0;
		dFloat m_radio1;
		dFloat m_height;
	} NewtonCylinderParam;

	typedef struct NewtonConeParam
	{
		dFloat m_radio;
		dFloat m_height;
	} NewtonConeParam;

	typedef struct NewtonChamferCylinderParam
	{
		dFloat m_radio;
		dFloat m_height;
	} NewtonChamferCylinderParam;

	typedef struct NewtonConvexHullParam
	{
		int m_vertexCount;
		int m_vertexStrideInBytes;
		int m_faceCount;
		dFloat* m_vertex;
	} NewtonConvexHullParam;

	typedef struct NewtonCompoundCollisionParam
	{
		int m_chidrenCount;
	} NewtonCompoundCollisionParam;

	typedef struct NewtonCollisionTreeParam
	{
		int m_vertexCount;
		int m_indexCount;
	} NewtonCollisionTreeParam;

	typedef struct NewtonDeformableMeshParam
	{
		int m_vertexCount;
		int m_triangleCount;
		int m_vrtexStrideInBytes;
		unsigned short *m_indexList;
		dFloat *m_vertexList;
	} NewtonDeformableMeshParam;

	typedef struct NewtonHeightFieldCollisionParam
	{
		int m_width;
		int m_height;
		int m_gridsDiagonals;
		int m_elevationDataType;	// 0 = 32 bit floats, 1 = unsigned 16 bit integers
		dFloat m_verticalScale;
		dFloat m_horizonalScale_x;
		dFloat m_horizonalScale_z;
		dFloat m_horizonalDisplacementScale_x;
		dFloat m_horizonalDisplacementScale_z;
		void* m_vertialElevation;
		short* m_horizotalDisplacement;
		char* m_atributes;
	} NewtonHeightFieldCollisionParam;

	typedef struct NewtonSceneCollisionParam
	{
		int m_childrenProxyCount;
	} NewtonSceneCollisionParam;

	typedef struct NewtonCollisionInfoRecord
	{
		dFloat m_offsetMatrix[4][4];
		NewtonCollisionMaterial m_collisionMaterial;
		int m_collisionType;				// tag id to identify the collision primitive
		union {
			NewtonBoxParam m_box;									
			NewtonConeParam m_cone;
			NewtonSphereParam m_sphere;
			NewtonCapsuleParam m_capsule;
			NewtonCylinderParam m_cylinder;
			NewtonChamferCylinderParam m_chamferCylinder;
			NewtonConvexHullParam m_convexHull;
			NewtonDeformableMeshParam m_deformableMesh;
			NewtonCompoundCollisionParam m_compoundCollision;
			NewtonCollisionTreeParam m_collisionTree;
			NewtonHeightFieldCollisionParam m_heightField;
			NewtonSceneCollisionParam m_sceneCollision;
			dFloat m_paramArray[64];		    // user define collision can use this to store information
		};
	} NewtonCollisionInfoRecord;

	typedef struct NewtonJointRecord
	{
		dFloat m_attachmenMatrix_0[4][4];
		dFloat m_attachmenMatrix_1[4][4];
		dFloat m_minLinearDof[3];
		dFloat m_maxLinearDof[3];
		dFloat m_minAngularDof[3];
		dFloat m_maxAngularDof[3];
		const NewtonBody* m_attachBody_0;
		const NewtonBody* m_attachBody_1;
		dFloat m_extraParameters[64];
		int	m_bodiesCollisionOn;
		char m_descriptionType[128];
	} NewtonJointRecord;

	typedef struct NewtonUserMeshCollisionCollideDesc
	{
		dFloat m_boxP0[4];							// lower bounding box of intersection query in local space
		dFloat m_boxP1[4];							// upper bounding box of intersection query in local space
		dFloat m_boxDistanceTravel[4];				// max distance that box bpxP0 and boxP1 can travel on this timestep, used this for continue collision mode.
		int m_threadNumber;							// current thread executing this query
		int	m_faceCount;                        	// the application should set here how many polygons intersect the query box
		int m_vertexStrideInBytes;              	// the application should set here the size of each vertex
		dFloat m_skinThickness;                     // this is the minimum skin separation specified by the material between these two colliding shapes
		void* m_userData;                       	// user data passed to the collision geometry at creation time

		NewtonBody* m_objBody;                  	// pointer to the colliding body
		NewtonBody* m_polySoupBody;             	// pointer to the rigid body owner of this collision tree 
		NewtonCollision* m_objCollision;			// collision shape of the colliding body, (no necessarily the collision of m_objBody)
		NewtonCollision* m_polySoupCollision;		// collision shape of the collision tree, (no necessarily the collision of m_polySoupBody)

		dFloat* m_vertex;                       	// the application should set here the pointer to the global vertex of the mesh. 
		int* m_faceIndexCount;                  	// the application should set here the pointer to the vertex count of each face.
		int* m_faceVertexIndex;                 	// the application should set here the pointer index array for each vertex on a face.
													// the format of a face is I0, I1, I2, I3, ..., M, N, E0, E1, E2, ..., A
                                                	// I0, I1, I2, .. are the indices to the vertex, relative to m_vertex pointer
		                                        	// M is the index to the material sub shape id
													// N in the index to the vertex normal relative to m_vertex pointer
													// E0, E1, E2, ... are the indices of the the face normal that is shared to that face edge, when the edge does not share a face normal then the edge index is set to index N, which the index to the face normal    
													// A is and estimate of the largest diagonal of the face, this used internally as a hint to improve floating point accuracy and algorithm performance. 
	} NewtonUserMeshCollisionCollideDesc;

	typedef struct NewtonWorldConvexCastReturnInfo
	{
		dFloat m_point[4];						// collision point in global space
		dFloat m_normal[4];						// surface normal at collision point in global space
		//dFloat m_normalOnHitPoint[4];           // surface normal at the surface of the hit body, 
												// is the same as the normal calculated by a ray cast hitting the body at the hit point
		dLong m_contactID;						// collision ID at contact point
		const NewtonBody* m_hitBody;			// body hit at contact point
		dFloat m_penetration;                   // contact penetration at collision point
	} NewtonWorldConvexCastReturnInfo;
	
	typedef struct NewtonUserMeshCollisionRayHitDesc
	{
		dFloat m_p0[4];							// ray origin in collision local space
		dFloat m_p1[4];                         // ray destination in collision local space   
		dFloat m_normalOut[4];					// copy here the normal at the ray intersection
		dLong m_userIdOut;						// copy here a user defined id for further feedback  
		void* m_userData;                       // user data passed to the collision geometry at creation time
	} NewtonUserMeshCollisionRayHitDesc;

	typedef struct NewtonHingeSliderUpdateDesc
	{
		dFloat m_accel;
		dFloat m_minFriction;
		dFloat m_maxFriction;
		dFloat m_timestep;
	} NewtonHingeSliderUpdateDesc;

	typedef struct NewtonUserContactPoint
	{
		dFloat m_point[4];
		dFloat m_normal[4];
		dLong m_shapeId0;
		dLong m_shapeId1;
		dFloat m_penetration;
		int m_unused[3];
	} NewtonUserContactPoint;

	// data structure for interfacing with NewtonMesh
	typedef struct NewtonMeshDoubleData
	{
		dFloat64* m_data;
		int* m_indexList;
		int m_strideInBytes;
	} NewtonMeshDoubleData;

	typedef struct NewtonMeshFloatData
	{
		dFloat* m_data;
		int* m_indexList;
		int m_strideInBytes;
	} NewtonMeshFloatData;

	typedef struct NewtonMeshVertexFormat
	{
		int m_faceCount;
		int* m_faceIndexCount;
		int* m_faceMaterial;
		NewtonMeshDoubleData m_vertex;
		NewtonMeshFloatData m_normal;
		NewtonMeshFloatData m_binormal;
		NewtonMeshFloatData m_uv0;
		NewtonMeshFloatData m_uv1;
		NewtonMeshFloatData m_vertexColor;
	} NewtonMeshVertexFormat;

    typedef struct dVector {
        dFloat  m_x;
        dFloat  m_y;
        dFloat  m_z;
        dFloat  m_w;
    } dVector;

    typedef struct dMatrix {
        dVector m_front;
        dVector m_up;
        dVector m_right;
        dVector m_posit;
    } dMatrix;

	typedef struct userData {
		dFloat 	radius;
		dFloat	mass;
		dFloat 	k1;
		dFloat 	k2;
	} userData;

	// Newton callback functions
	typedef void* (*NewtonAllocMemory) (int sizeInBytes);
	typedef void (*NewtonFreeMemory) (void* const ptr, int sizeInBytes);

	typedef void (*NewtonWorldDestructorCallback) (const NewtonWorld* const world);
	typedef void (*NewtonPostUpdateCallback) (const NewtonWorld* const world, dFloat timestep);

	typedef void (*NewtonWorldListenerDebugCallback) (const NewtonWorld* const world, void* const listener, void* const debugContext);
	typedef void (*NewtonWorldListenerBodyDestroyCallback) (const NewtonWorld* const world, void* const listenerUserData, NewtonBody* const body);
	typedef void (*NewtonWorldUpdateListenerCallback) (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep);
	typedef void (*NewtonWorldDestroyListenerCallback) (const NewtonWorld* const world, void* const listenerUserData);

	typedef dLong (*NewtonGetTimeInMicrosencondsCallback) ();

	typedef void (*NewtonSerializeCallback) (void* const serializeHandle, const void* const buffer, int size);
	typedef void (*NewtonDeserializeCallback) (void* const serializeHandle, void* const buffer, int size);
	
	typedef void (*NewtonOnBodySerializationCallback) (NewtonBody* const body, void* const userData, NewtonSerializeCallback function, void* const serializeHandle);
	typedef void (*NewtonOnBodyDeserializationCallback) (NewtonBody* const body, void* const userData, NewtonDeserializeCallback function, void* const serializeHandle);

	typedef void (*NewtonOnJointSerializationCallback) (const NewtonJoint* const joint, NewtonSerializeCallback function, void* const serializeHandle);
	typedef void (*NewtonOnJointDeserializationCallback) (NewtonBody* const body0, NewtonBody* const body1, NewtonDeserializeCallback function, void* const serializeHandle);

	typedef void (*NewtonOnUserCollisionSerializationCallback) (void* const userData, NewtonSerializeCallback function, void* const serializeHandle);
	
	// user collision callbacks	
	typedef void (*NewtonUserMeshCollisionDestroyCallback) (void* const userData);
	typedef dFloat (*NewtonUserMeshCollisionRayHitCallback) (NewtonUserMeshCollisionRayHitDesc* const lineDescData);
	typedef void (*NewtonUserMeshCollisionGetCollisionInfo) (void* const userData, NewtonCollisionInfoRecord* const infoRecord);
	typedef int (*NewtonUserMeshCollisionAABBTest) (void* const userData, const dFloat* const boxP0, const dFloat* const boxP1);
	typedef int (*NewtonUserMeshCollisionGetFacesInAABB) (void* const userData, const dFloat* const p0, const dFloat* const p1,
														   const dFloat** const vertexArray, int* const vertexCount, int* const vertexStrideInBytes, 
		                                                   const int* const indexList, int maxIndexCount, const int* const userDataList);
	typedef void (*NewtonUserMeshCollisionCollideCallback) (NewtonUserMeshCollisionCollideDesc* const collideDescData, const void* const continueCollisionHandle);

	typedef int (*NewtonTreeCollisionFaceCallback) (void* const context, const dFloat* const polygon, int strideInBytes, const int* const indexArray, int indexCount);

	typedef dFloat (*NewtonCollisionTreeRayCastCallback) (const NewtonBody* const body, const NewtonCollision* const treeCollision, dFloat intersection, dFloat* const normal, int faceId, void* const usedData);
	typedef dFloat (*NewtonHeightFieldRayCastCallback) (const NewtonBody* const body, const NewtonCollision* const heightFieldCollision, dFloat intersection, int row, int col, dFloat* const normal, int faceId, void* const usedData);

	typedef void (*NewtonCollisionCopyConstructionCallback) (const NewtonWorld* const newtonWorld, NewtonCollision* const collision, const NewtonCollision* const sourceCollision);
	typedef void (*NewtonCollisionDestructorCallback) (const NewtonWorld* const newtonWorld, const NewtonCollision* const collision);

	// collision tree call back (obsoleted no recommended)
	typedef void (*NewtonTreeCollisionCallback) (const NewtonBody* const bodyWithTreeCollision, const NewtonBody* const body, int faceID, 
												 int vertexCount, const dFloat* const vertex, int vertexStrideInBytes); 

	typedef void (*NewtonBodyDestructor) (const NewtonBody* const body);
	typedef void (*NewtonApplyForceAndTorque) (const NewtonBody* const body, dFloat timestep, int threadIndex);
	typedef void (*NewtonSetTransform) (const NewtonBody* const body, const dFloat* const matrix, int threadIndex);

	typedef int (*NewtonIslandUpdate) (const NewtonWorld* const newtonWorld, const void* islandHandle, int bodyCount);
	
	typedef void (*NewtonFractureCompoundCollisionOnEmitCompoundFractured) (NewtonBody* const fracturedBody);
	typedef void (*NewtonFractureCompoundCollisionOnEmitChunk) (NewtonBody* const chunkBody, NewtonFracturedCompoundMeshPart* const fracturexChunkMesh, const NewtonCollision* const fracturedCompountCollision);
	typedef void (*NewtonFractureCompoundCollisionReconstructMainMeshCallBack) (NewtonBody* const body, NewtonFracturedCompoundMeshPart* const mainMesh, const NewtonCollision* const fracturedCompountCollision);

	typedef unsigned (*NewtonWorldRayPrefilterCallback)(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData);
	typedef dFloat (*NewtonWorldRayFilterCallback)(const NewtonBody* const body, const NewtonCollision* const shapeHit, const dFloat* const hitContact, const dFloat* const hitNormal, dLong collisionID, void* const userData, dFloat intersectParam);

	typedef int (*NewtonOnAABBOverlap) (const NewtonJoint* const contact, dFloat timestep, int threadIndex);
	typedef void (*NewtonContactsProcess) (const NewtonJoint* const contact, dFloat timestep, int threadIndex);
//	typedef int  (*NewtonOnAABBOverlap) (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex);
//	typedef int  (*NewtonOnCompoundSubCollisionAABBOverlap) (const NewtonMaterial* const material, const NewtonBody* const body0, const void* const collisionNode0, const NewtonBody* const body1, const void* const collisionNode1, int threadIndex);
	typedef int (*NewtonOnCompoundSubCollisionAABBOverlap) (const NewtonJoint* const contact, dFloat timestep, const NewtonBody* const body0, const void* const collisionNode0, const NewtonBody* const body1, const void* const collisionNode1, int threadIndex);
	typedef int  (*NewtonOnContactGeneration) (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonCollision* const collision0, const NewtonBody* const body1, const NewtonCollision* const collision1, NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex);

	typedef int (*NewtonBodyIterator) (const NewtonBody* const body, void* const userData);
	typedef void (*NewtonJointIterator) (const NewtonJoint* const joint, void* const userData);
	typedef void (*NewtonCollisionIterator) (void* const userData, int vertexCount, const dFloat* const faceArray, int faceId);

	typedef void (*NewtonBallCallback) (const NewtonJoint* const ball, dFloat timestep);
	typedef unsigned (*NewtonHingeCallback) (const NewtonJoint* const hinge, NewtonHingeSliderUpdateDesc* const desc);
	typedef unsigned (*NewtonSliderCallback) (const NewtonJoint* const slider, NewtonHingeSliderUpdateDesc* const desc);
	typedef unsigned (*NewtonUniversalCallback) (const NewtonJoint* const universal, NewtonHingeSliderUpdateDesc* const desc);
	typedef unsigned (*NewtonCorkscrewCallback) (const NewtonJoint* const corkscrew, NewtonHingeSliderUpdateDesc* const desc);

	typedef void (*NewtonUserBilateralCallback) (const NewtonJoint* const userJoint, dFloat timestep, int threadIndex);
	typedef void (*NewtonUserBilateralGetInfoCallback) (const NewtonJoint* const userJoint, NewtonJointRecord* const info);

	typedef void (*NewtonConstraintDestructor) (const NewtonJoint* const me);

	typedef void (*NewtonJobTask) (NewtonWorld* const world, void* const userData, int threadIndex);
	typedef int (*NewtonReportProgress) (dFloat normalizedProgressPercent, void* const userData);

	// **********************************************************************************************
	//
	// world control functions
	//
	// **********************************************************************************************
	 int NewtonWorldGetVersion ();
	 int NewtonWorldFloatSize ();

	 int NewtonGetMemoryUsed ();
	 void NewtonSetMemorySystem (NewtonAllocMemory malloc, NewtonFreeMemory free);

	 NewtonWorld* NewtonCreate ();
	 void NewtonDestroy (const NewtonWorld* const newtonWorld);
	 void NewtonDestroyAllBodies (const NewtonWorld* const newtonWorld);

	 void NewtonSetPosUpdateCallback (const NewtonWorld* const newtonWorld, NewtonPostUpdateCallback callback);

	 void* NewtonAlloc (int sizeInBytes);
	 void NewtonFree (void* const ptr);

	 void* NewtonCurrentPlugin(const NewtonWorld* const newtonWorld);
	 void* NewtonGetFirstPlugin(const NewtonWorld* const newtonWorld);
	 void* NewtonGetPreferedPlugin(const NewtonWorld* const newtonWorld);
	 void* NewtonGetNextPlugin(const NewtonWorld* const newtonWorld, const void* const plugin);
	 const char* NewtonGetPluginString(const NewtonWorld* const newtonWorld, const void* const plugin);
	 void NewtonSelectPlugin(const NewtonWorld* const newtonWorld, const void* const plugin);

	 dFloat NewtonGetContactMergeTolerance (const NewtonWorld* const newtonWorld);
	 void NewtonSetContactMergeTolerance (const NewtonWorld* const newtonWorld, dFloat tolerance);

	 void NewtonInvalidateCache (const NewtonWorld* const newtonWorld);

	 void NewtonSetSolverModel (const NewtonWorld* const newtonWorld, int model);
	 int NewtonGetSolverModel(const NewtonWorld* const newtonWorld);

	 void NewtonSetMultiThreadSolverOnSingleIsland (const NewtonWorld* const newtonWorld, int mode);
	 int NewtonGetMultiThreadSolverOnSingleIsland (const NewtonWorld* const newtonWorld);

	 int NewtonGetBroadphaseAlgorithm (const NewtonWorld* const newtonWorld);
	 void NewtonSelectBroadphaseAlgorithm (const NewtonWorld* const newtonWorld, int algorithmType);
	 void NewtonResetBroadphase(const NewtonWorld* const newtonWorld);
	
	 void NewtonUpdate (const NewtonWorld* const newtonWorld, dFloat timestep);
	 void NewtonUpdateAsync (const NewtonWorld* const newtonWorld, dFloat timestep);
	 void NewtonWaitForUpdateToFinish (const NewtonWorld* const newtonWorld);

	 int NewtonGetNumberOfSubsteps (const NewtonWorld* const newtonWorld);
	 void NewtonSetNumberOfSubsteps (const NewtonWorld* const newtonWorld, int subSteps);
	 dFloat NewtonGetLastUpdateTime (const NewtonWorld* const newtonWorld);

	 void NewtonSerializeToFile (const NewtonWorld* const newtonWorld, const char* const filename, NewtonOnBodySerializationCallback bodyCallback, void* const bodyUserData);
	 void NewtonDeserializeFromFile (const NewtonWorld* const newtonWorld, const char* const filename, NewtonOnBodyDeserializationCallback bodyCallback, void* const bodyUserData);

	 void NewtonSerializeScene(const NewtonWorld* const newtonWorld, NewtonOnBodySerializationCallback bodyCallback, void* const bodyUserData,
									   	 NewtonSerializeCallback serializeCallback, void* const serializeHandle);
	 void NewtonDeserializeScene(const NewtonWorld* const newtonWorld, NewtonOnBodyDeserializationCallback bodyCallback, void* const bodyUserData,
										   NewtonDeserializeCallback serializeCallback, void* const serializeHandle);

	 NewtonBody* NewtonFindSerializedBody(const NewtonWorld* const newtonWorld, int bodySerializedID);
	 void NewtonSetJointSerializationCallbacks (const NewtonWorld* const newtonWorld, NewtonOnJointSerializationCallback serializeJoint, NewtonOnJointDeserializationCallback deserializeJoint);
	 void NewtonGetJointSerializationCallbacks (const NewtonWorld* const newtonWorld, NewtonOnJointSerializationCallback* const serializeJoint, NewtonOnJointDeserializationCallback* const deserializeJoint);

	// multi threading interface 
	 void NewtonWorldCriticalSectionLock (const NewtonWorld* const newtonWorld, int threadIndex);
	 void NewtonWorldCriticalSectionUnlock (const NewtonWorld* const newtonWorld);
	 void NewtonSetThreadsCount (const NewtonWorld* const newtonWorld, int threads);
	 int NewtonGetThreadsCount(const NewtonWorld* const newtonWorld);
	 int NewtonGetMaxThreadsCount(const NewtonWorld* const newtonWorld);
	 void NewtonDispachThreadJob(const NewtonWorld* const newtonWorld, NewtonJobTask task, void* const usedData);
	 void NewtonSyncThreadJobs(const NewtonWorld* const newtonWorld);

	// atomic operations
	 int NewtonAtomicAdd (int* const ptr, int value);
	 int NewtonAtomicSwap (int* const ptr, int value);
	 void NewtonYield ();

	 void NewtonSetIslandUpdateEvent (const NewtonWorld* const newtonWorld, NewtonIslandUpdate islandUpdate); 
//	 void NewtonSetDestroyBodyByExeciveForce (const NewtonWorld* const newtonWorld, NewtonDestroyBodyByExeciveForce callback); 
//	 void NewtonWorldForEachBodyDo (const NewtonWorld* const newtonWorld, NewtonBodyIterator callback);
	 void NewtonWorldForEachJointDo (const NewtonWorld* const newtonWorld, NewtonJointIterator callback, void* const userData);
	 void NewtonWorldForEachBodyInAABBDo (const NewtonWorld* const newtonWorld, const dFloat* const p0, const dFloat* const p1, NewtonBodyIterator callback, void* const userData);

	 void NewtonWorldSetUserData (const NewtonWorld* const newtonWorld, void* const userData);
	 void* NewtonWorldGetUserData (const NewtonWorld* const newtonWorld);
	
	 void* NewtonWorldAddListener (const NewtonWorld* const newtonWorld, const char* const nameId, void* const listenerUserData);
	 void* NewtonWorldGetListener (const NewtonWorld* const newtonWorld, const char* const nameId);

	 void NewtonWorldListenerSetDestructorCallback (const NewtonWorld* const newtonWorld, void* const listener, NewtonWorldDestroyListenerCallback destroy);
	 void NewtonWorldListenerSetPreUpdateCallback (const NewtonWorld* const newtonWorld, void* const listener, NewtonWorldUpdateListenerCallback update);
	 void NewtonWorldListenerSetPostUpdateCallback (const NewtonWorld* const newtonWorld, void* const listener, NewtonWorldUpdateListenerCallback update);
	 void NewtonWorldListenerSetDebugCallback (const NewtonWorld* const newtonWorld, void* const listener, NewtonWorldListenerDebugCallback debugCallback);
	 void NewtonWorldListenerSetBodyDestroyCallback(const NewtonWorld* const newtonWorld, void* const listener, NewtonWorldListenerBodyDestroyCallback bodyDestroyCallback);
	 void NewtonWorldListenerDebug(const NewtonWorld* const newtonWorld, void* const context);
	 void* NewtonWorldGetListenerUserData(const NewtonWorld* const newtonWorld, void* const listener);
	 NewtonWorldListenerBodyDestroyCallback NewtonWorldListenerGetBodyDestroyCallback (const NewtonWorld* const newtonWorld, void* const listener);


	 void NewtonWorldSetDestructorCallback (const NewtonWorld* const newtonWorld, NewtonWorldDestructorCallback destructor);
	 NewtonWorldDestructorCallback NewtonWorldGetDestructorCallback (const NewtonWorld* const newtonWorld);
	 void NewtonWorldSetCollisionConstructorDestructorCallback (const NewtonWorld* const newtonWorld, NewtonCollisionCopyConstructionCallback constructor, NewtonCollisionDestructorCallback destructor);

	 void NewtonWorldRayCast (const NewtonWorld* const newtonWorld, const dFloat* const p0, const dFloat* const p1, NewtonWorldRayFilterCallback filter, void* const userData, NewtonWorldRayPrefilterCallback prefilter, int threadIndex);
	 int NewtonWorldConvexCast (const NewtonWorld* const newtonWorld, const dFloat* const matrix, const dFloat* const target, const NewtonCollision* const shape, dFloat* const param, void* const userData, NewtonWorldRayPrefilterCallback prefilter, NewtonWorldConvexCastReturnInfo* const info, int maxContactsCount, int threadIndex);
	 int NewtonWorldCollide (const NewtonWorld* const newtonWorld, const dFloat* const matrix, const NewtonCollision* const shape, void* const userData, NewtonWorldRayPrefilterCallback prefilter, NewtonWorldConvexCastReturnInfo* const info, int maxContactsCount, int threadIndex);
	
	// world utility functions
	 int NewtonWorldGetBodyCount(const NewtonWorld* const newtonWorld);
	 int NewtonWorldGetConstraintCount(const NewtonWorld* const newtonWorld);

	// **********************************************************************************************
	//
	// Simulation islands 
	//
	// **********************************************************************************************
	 NewtonBody* NewtonIslandGetBody (const void* const island, int bodyIndex);
	 void NewtonIslandGetBodyAABB (const void* const island, int bodyIndex, dFloat* const p0, dFloat* const p1);

	// **********************************************************************************************
	//
	// Physics Material Section
	//
	// **********************************************************************************************
	 int NewtonMaterialCreateGroupID(const NewtonWorld* const newtonWorld);
	 int NewtonMaterialGetDefaultGroupID(const NewtonWorld* const newtonWorld);
	 void NewtonMaterialDestroyAllGroupID(const NewtonWorld* const newtonWorld);

	// material definitions that can not be overwritten in function callback
	 void* NewtonMaterialGetUserData (const NewtonWorld* const newtonWorld, int id0, int id1);
	 void NewtonMaterialSetSurfaceThickness (const NewtonWorld* const newtonWorld, int id0, int id1, dFloat thickness);

//	deprecated, not longer continue collision is set on the material  	
//	 void NewtonMaterialSetContinuousCollisionMode (const NewtonWorld* const newtonWorld, int id0, int id1, int state);
	
	 void NewtonMaterialSetCallbackUserData (const NewtonWorld* const newtonWorld, int id0, int id1, void* const userData);
	 void NewtonMaterialSetContactGenerationCallback (const NewtonWorld* const newtonWorld, int id0, int id1, NewtonOnContactGeneration contactGeneration);
	 void NewtonMaterialSetCompoundCollisionCallback(const NewtonWorld* const newtonWorld, int id0, int id1, NewtonOnCompoundSubCollisionAABBOverlap compoundAabbOverlap);
	 void NewtonMaterialSetCollisionCallback (const NewtonWorld* const newtonWorld, int id0, int id1, NewtonOnAABBOverlap aabbOverlap, NewtonContactsProcess process);

	 void NewtonMaterialSetDefaultSoftness (const NewtonWorld* const newtonWorld, int id0, int id1, dFloat value);
	 void NewtonMaterialSetDefaultElasticity (const NewtonWorld* const newtonWorld, int id0, int id1, dFloat elasticCoef);
	 void NewtonMaterialSetDefaultCollidable (const NewtonWorld* const newtonWorld, int id0, int id1, int state);
	 void NewtonMaterialSetDefaultFriction (const NewtonWorld* const newtonWorld, int id0, int id1, dFloat staticFriction, dFloat kineticFriction);

	 NewtonMaterial* NewtonWorldGetFirstMaterial (const NewtonWorld* const newtonWorld);
	 NewtonMaterial* NewtonWorldGetNextMaterial (const NewtonWorld* const newtonWorld, const NewtonMaterial* const material);

	 NewtonBody* NewtonWorldGetFirstBody (const NewtonWorld* const newtonWorld);
	 NewtonBody* NewtonWorldGetNextBody (const NewtonWorld* const newtonWorld, const NewtonBody* const curBody);


	// **********************************************************************************************
	//
	// Physics Contact control functions
	//
	// **********************************************************************************************
	 void *NewtonMaterialGetMaterialPairUserData (const NewtonMaterial* const material);
	 unsigned NewtonMaterialGetContactFaceAttribute (const NewtonMaterial* const material);
	 NewtonCollision* NewtonMaterialGetBodyCollidingShape (const NewtonMaterial* const material, const NewtonBody* const body);
	 dFloat NewtonMaterialGetContactNormalSpeed (const NewtonMaterial* const material);
	 void NewtonMaterialGetContactForce (const NewtonMaterial* const material, const NewtonBody* const body, dFloat* const force);
	 void NewtonMaterialGetContactPositionAndNormal (const NewtonMaterial* const material, const NewtonBody* const body, dFloat* const posit, dFloat* const normal);
	 void NewtonMaterialGetContactTangentDirections (const NewtonMaterial* const material, const NewtonBody* const body, dFloat* const dir0, dFloat* const dir1);
	 dFloat NewtonMaterialGetContactTangentSpeed (const NewtonMaterial* const material, int index);
	 dFloat NewtonMaterialGetContactMaxNormalImpact (const NewtonMaterial* const material);
	 dFloat NewtonMaterialGetContactMaxTangentImpact (const NewtonMaterial* const material, int index);
	 dFloat NewtonMaterialGetContactPenetration (const NewtonMaterial* const material);
		
	 void NewtonMaterialSetContactSoftness (const NewtonMaterial* const material, dFloat softness);
	 void NewtonMaterialSetContactThickness (const NewtonMaterial* const material, dFloat thickness);
	 void NewtonMaterialSetContactElasticity (const NewtonMaterial* const material, dFloat restitution);
	 void NewtonMaterialSetContactFrictionState (const NewtonMaterial* const material, int state, int index);
	 void NewtonMaterialSetContactFrictionCoef (const NewtonMaterial* const material, dFloat staticFrictionCoef, dFloat kineticFrictionCoef, int index);
	
	 void NewtonMaterialSetContactNormalAcceleration (const NewtonMaterial* const material, dFloat accel);
	 void NewtonMaterialSetContactNormalDirection (const NewtonMaterial* const material, const dFloat* const directionVector);
	 void NewtonMaterialSetContactPosition (const NewtonMaterial* const material, const dFloat* const position);

	 void NewtonMaterialSetContactTangentFriction (const NewtonMaterial* const material, dFloat friction, int index);
	 void NewtonMaterialSetContactTangentAcceleration (const NewtonMaterial* const material, dFloat accel, int index);
	 void NewtonMaterialContactRotateTangentDirections (const NewtonMaterial* const material, const dFloat* const directionVector);

	// dFloat NewtonMaterialGetContactPruningTolerance (const NewtonBody* const body0, const NewtonBody* const body1);
	// void NewtonMaterialSetContactPruningTolerance (const NewtonBody* const body0, const NewtonBody* const body1, dFloat tolerance);
	 dFloat NewtonMaterialGetContactPruningTolerance(const NewtonJoint* const contactJoint);
	 void NewtonMaterialSetContactPruningTolerance(const NewtonJoint* const contactJoint, dFloat tolerance);

	// **********************************************************************************************
	//
	// convex collision primitives creation functions
	//
	// **********************************************************************************************
	 NewtonCollision* NewtonCreateNull (const NewtonWorld* const newtonWorld);
	 NewtonCollision* NewtonCreateSphere (const NewtonWorld* const newtonWorld, dFloat radius, int shapeID, const dFloat* const offsetMatrix);
	 NewtonCollision* NewtonCreateBox (const NewtonWorld* const newtonWorld, dFloat dx, dFloat dy, dFloat dz, int shapeID, const dFloat* const offsetMatrix);
	 NewtonCollision* NewtonCreateCone (const NewtonWorld* const newtonWorld, dFloat radius, dFloat height, int shapeID, const dFloat* const offsetMatrix);
	 NewtonCollision* NewtonCreateCapsule (const NewtonWorld* const newtonWorld, dFloat radius0, dFloat radius1, dFloat height, int shapeID, const dFloat* const offsetMatrix);
	 NewtonCollision* NewtonCreateCylinder (const NewtonWorld* const newtonWorld, dFloat radio0, dFloat radio1, dFloat height, int shapeID, const dFloat* const offsetMatrix);
	 NewtonCollision* NewtonCreateChamferCylinder (const NewtonWorld* const newtonWorld, dFloat radius, dFloat height, int shapeID, const dFloat* const offsetMatrix);
	 NewtonCollision* NewtonCreateConvexHull (const NewtonWorld* const newtonWorld, int count, const dFloat* const vertexCloud, int strideInBytes, dFloat tolerance, int shapeID, const dFloat* const offsetMatrix);
	 NewtonCollision* NewtonCreateConvexHullFromMesh (const NewtonWorld* const newtonWorld, const NewtonMesh* const mesh, dFloat tolerance, int shapeID);

	 int NewtonCollisionGetMode(const NewtonCollision* const convexCollision);
	 void NewtonCollisionSetMode (const NewtonCollision* const convexCollision, int mode);

//	 void NewtonCollisionSetMaxBreakImpactImpulse(const NewtonCollision* const convexHullCollision, dFloat maxImpactImpulse);
//	 dFloat NewtonCollisionGetMaxBreakImpactImpulse(const NewtonCollision* const convexHullCollision);

	 int NewtonConvexHullGetFaceIndices (const NewtonCollision* const convexHullCollision, int face, int* const faceIndices);
	 int NewtonConvexHullGetVertexData (const NewtonCollision* const convexHullCollision, dFloat** const vertexData, int* strideInBytes);
	
	 dFloat NewtonConvexCollisionCalculateVolume (const NewtonCollision* const convexCollision);
	 void NewtonConvexCollisionCalculateInertialMatrix (const NewtonCollision* convexCollision, dFloat* const inertia, dFloat* const origin);	
	 void NewtonConvexCollisionCalculateBuoyancyAcceleration (const NewtonCollision* const convexCollision, const dFloat* const matrix, const dFloat* const shapeOrigin, const dFloat* const gravityVector, const dFloat* const fluidPlane, dFloat fluidDensity, dFloat fluidViscosity, dFloat* const accel, dFloat* const alpha);

	 const void* NewtonCollisionDataPointer (const NewtonCollision* const convexCollision);

	// **********************************************************************************************
	//
	// compound collision primitives creation functions
	//
	// **********************************************************************************************
	 NewtonCollision* NewtonCreateCompoundCollision (const NewtonWorld* const newtonWorld, int shapeID);
	 NewtonCollision* NewtonCreateCompoundCollisionFromMesh (const NewtonWorld* const newtonWorld, const NewtonMesh* const mesh, dFloat hullTolerance, int shapeID, int subShapeID);

	 void NewtonCompoundCollisionBeginAddRemove (NewtonCollision* const compoundCollision);	
	 void* NewtonCompoundCollisionAddSubCollision (NewtonCollision* const compoundCollision, const NewtonCollision* const convexCollision);	
	 void NewtonCompoundCollisionRemoveSubCollision (NewtonCollision* const compoundCollision, const void* const collisionNode);	
	 void NewtonCompoundCollisionRemoveSubCollisionByIndex (NewtonCollision* const compoundCollision, int nodeIndex);	
	 void NewtonCompoundCollisionSetSubCollisionMatrix (NewtonCollision* const compoundCollision, const void* const collisionNode, const dFloat* const matrix);	
	 void NewtonCompoundCollisionEndAddRemove (NewtonCollision* const compoundCollision);	

	 void* NewtonCompoundCollisionGetFirstNode (NewtonCollision* const compoundCollision);
	 void* NewtonCompoundCollisionGetNextNode (NewtonCollision* const compoundCollision, const void* const collisionNode);

	 void* NewtonCompoundCollisionGetNodeByIndex (NewtonCollision* const compoundCollision, int index);
	 int NewtonCompoundCollisionGetNodeIndex (NewtonCollision* const compoundCollision, const void* const collisionNode);
	 NewtonCollision* NewtonCompoundCollisionGetCollisionFromNode (NewtonCollision* const compoundCollision, const void* const collisionNode);


	// **********************************************************************************************
	//
	// Fractured compound collision primitives interface
	//
	// **********************************************************************************************
	 NewtonCollision* NewtonCreateFracturedCompoundCollision (const NewtonWorld* const newtonWorld, const NewtonMesh* const solidMesh, int shapeID, int fracturePhysicsMaterialID, int pointcloudCount, const dFloat* const vertexCloud, int strideInBytes, int materialID, const dFloat* const textureMatrix,
																		NewtonFractureCompoundCollisionReconstructMainMeshCallBack regenerateMainMeshCallback, 
																		NewtonFractureCompoundCollisionOnEmitCompoundFractured emitFracturedCompound, NewtonFractureCompoundCollisionOnEmitChunk emitFracfuredChunk);
	 NewtonCollision* NewtonFracturedCompoundPlaneClip (const NewtonCollision* const fracturedCompound, const dFloat* const plane);

	 void NewtonFracturedCompoundSetCallbacks (const NewtonCollision* const fracturedCompound, NewtonFractureCompoundCollisionReconstructMainMeshCallBack regenerateMainMeshCallback, 
														 NewtonFractureCompoundCollisionOnEmitCompoundFractured emitFracturedCompound, NewtonFractureCompoundCollisionOnEmitChunk emitFracfuredChunk);


	 int NewtonFracturedCompoundIsNodeFreeToDetach (const NewtonCollision* const fracturedCompound, void* const collisionNode);
	 int NewtonFracturedCompoundNeighborNodeList (const NewtonCollision* const fracturedCompound, void* const collisionNode, void** const list, int maxCount);

	
	 NewtonFracturedCompoundMeshPart* NewtonFracturedCompoundGetMainMesh (const NewtonCollision* const fracturedCompound);
	 NewtonFracturedCompoundMeshPart* NewtonFracturedCompoundGetFirstSubMesh(const NewtonCollision* const fracturedCompound);
	 NewtonFracturedCompoundMeshPart* NewtonFracturedCompoundGetNextSubMesh(const NewtonCollision* const fracturedCompound, NewtonFracturedCompoundMeshPart* const subMesh);

	 int NewtonFracturedCompoundCollisionGetVertexCount (const NewtonCollision* const fracturedCompound, const NewtonFracturedCompoundMeshPart* const meshOwner); 
	 const dFloat* NewtonFracturedCompoundCollisionGetVertexPositions (const NewtonCollision* const fracturedCompound, const NewtonFracturedCompoundMeshPart* const meshOwner);
	 const dFloat* NewtonFracturedCompoundCollisionGetVertexNormals (const NewtonCollision* const fracturedCompound, const NewtonFracturedCompoundMeshPart* const meshOwner);
	 const dFloat* NewtonFracturedCompoundCollisionGetVertexUVs (const NewtonCollision* const fracturedCompound, const NewtonFracturedCompoundMeshPart* const meshOwner);
	 int NewtonFracturedCompoundMeshPartGetIndexStream (const NewtonCollision* const fracturedCompound, const NewtonFracturedCompoundMeshPart* const meshOwner, const void* const segment, int* const index); 

	 void* NewtonFracturedCompoundMeshPartGetFirstSegment (const NewtonFracturedCompoundMeshPart* const fractureCompoundMeshPart); 
	 void* NewtonFracturedCompoundMeshPartGetNextSegment (const void* const fractureCompoundMeshSegment); 
	 int NewtonFracturedCompoundMeshPartGetMaterial (const void* const fractureCompoundMeshSegment); 
	 int NewtonFracturedCompoundMeshPartGetIndexCount (const void* const fractureCompoundMeshSegment); 


	// **********************************************************************************************
	//
	// scene collision are static compound collision that can take polygonal static collisions
	//
	// **********************************************************************************************
	 NewtonCollision* NewtonCreateSceneCollision (const NewtonWorld* const newtonWorld, int shapeID);

	 void NewtonSceneCollisionBeginAddRemove (NewtonCollision* const sceneCollision);	
	 void* NewtonSceneCollisionAddSubCollision (NewtonCollision* const sceneCollision, const NewtonCollision* const collision);	
	 void NewtonSceneCollisionRemoveSubCollision (NewtonCollision* const compoundCollision, const void* const collisionNode);	
	 void NewtonSceneCollisionRemoveSubCollisionByIndex (NewtonCollision* const sceneCollision, int nodeIndex);
	 void NewtonSceneCollisionSetSubCollisionMatrix (NewtonCollision* const sceneCollision, const void* const collisionNode, const dFloat* const matrix);	
	 void NewtonSceneCollisionEndAddRemove (NewtonCollision* const sceneCollision);	

	 void* NewtonSceneCollisionGetFirstNode (NewtonCollision* const sceneCollision);
	 void* NewtonSceneCollisionGetNextNode (NewtonCollision* const sceneCollision, const void* const collisionNode);

	 void* NewtonSceneCollisionGetNodeByIndex (NewtonCollision* const sceneCollision, int index);
	 int NewtonSceneCollisionGetNodeIndex (NewtonCollision* const sceneCollision, const void* const collisionNode);
	 NewtonCollision* NewtonSceneCollisionGetCollisionFromNode (NewtonCollision* const sceneCollision, const void* const collisionNode);


	//  ***********************************************************************************************************
	//
	//	User Static mesh collision interface
	//
	// ***********************************************************************************************************
	 NewtonCollision* NewtonCreateUserMeshCollision (const NewtonWorld* const newtonWorld, const dFloat* const minBox, 
		const dFloat* const maxBox, void* const userData, NewtonUserMeshCollisionCollideCallback collideCallback, 
		NewtonUserMeshCollisionRayHitCallback rayHitCallback, NewtonUserMeshCollisionDestroyCallback destroyCallback,
		NewtonUserMeshCollisionGetCollisionInfo getInfoCallback, NewtonUserMeshCollisionAABBTest getLocalAABBCallback, 
		NewtonUserMeshCollisionGetFacesInAABB facesInAABBCallback, NewtonOnUserCollisionSerializationCallback serializeCallback, int shapeID);

	 int NewtonUserMeshCollisionContinuousOverlapTest (const NewtonUserMeshCollisionCollideDesc* const collideDescData, const void* const continueCollisionHandle, const dFloat* const minAabb, const dFloat* const maxAabb);
	
	//  ***********************************************************************************************************
	//
	//	Collision serialization functions
	//
	// ***********************************************************************************************************
	 NewtonCollision* NewtonCreateCollisionFromSerialization (const NewtonWorld* const newtonWorld, NewtonDeserializeCallback deserializeFunction, void* const serializeHandle);
	 void NewtonCollisionSerialize (const NewtonWorld* const newtonWorld, const NewtonCollision* const collision, NewtonSerializeCallback serializeFunction, void* const serializeHandle);
	 void NewtonCollisionGetInfo (const NewtonCollision* const collision, NewtonCollisionInfoRecord* const collisionInfo);

	// **********************************************************************************************
	//
	// Static collision shapes functions
	//
	// **********************************************************************************************
	 NewtonCollision* NewtonCreateHeightFieldCollision (const NewtonWorld* const newtonWorld, int width, int height, int gridsDiagonals, int elevationdatType, const void* const elevationMap, const char* const attributeMap, dFloat verticalScale, dFloat horizontalScale_x, dFloat horizontalScale_z, int shapeID);
	 void NewtonHeightFieldSetUserRayCastCallback (const NewtonCollision* const heightfieldCollision, NewtonHeightFieldRayCastCallback rayHitCallback);
	 void NewtonHeightFieldSetHorizontalDisplacement (const NewtonCollision* const heightfieldCollision, const unsigned short* const horizontalMap, dFloat scale);

	 NewtonCollision* NewtonCreateTreeCollision (const NewtonWorld* const newtonWorld, int shapeID);
	 NewtonCollision* NewtonCreateTreeCollisionFromMesh (const NewtonWorld* const newtonWorld, const NewtonMesh* const mesh, int shapeID);
	 void NewtonTreeCollisionSetUserRayCastCallback (const NewtonCollision* const treeCollision, NewtonCollisionTreeRayCastCallback rayHitCallback);

	 void NewtonTreeCollisionBeginBuild (const NewtonCollision* const treeCollision);
	 void NewtonTreeCollisionAddFace (const NewtonCollision* const treeCollision, int vertexCount, const dFloat* const vertexPtr, int strideInBytes, int faceAttribute);
	 void NewtonTreeCollisionEndBuild (const NewtonCollision* const treeCollision, int optimize);

	 int NewtonTreeCollisionGetFaceAttribute (const NewtonCollision* const treeCollision, const int* const faceIndexArray, int indexCount); 
	 void NewtonTreeCollisionSetFaceAttribute (const NewtonCollision* const treeCollision, const int* const faceIndexArray, int indexCount, int attribute);

	 void NewtonTreeCollisionForEachFace (const NewtonCollision* const treeCollision, NewtonTreeCollisionFaceCallback forEachFaceCallback, void* const context); 

	 int NewtonTreeCollisionGetVertexListTriangleListInAABB (const NewtonCollision* const treeCollision, const dFloat* const p0, const dFloat* const p1, const dFloat** const vertexArray, int* const vertexCount, int* const vertexStrideInBytes, const int* const indexList, int maxIndexCount, const int* const faceAttribute); 

	 void NewtonStaticCollisionSetDebugCallback (const NewtonCollision* const staticCollision, NewtonTreeCollisionCallback userCallback);

	// **********************************************************************************************
	//
	// General purpose collision library functions
	//
	// **********************************************************************************************
	 NewtonCollision* NewtonCollisionCreateInstance (const NewtonCollision* const collision);
	 int NewtonCollisionGetType (const NewtonCollision* const collision);
	 int NewtonCollisionIsConvexShape (const NewtonCollision* const collision);
	 int NewtonCollisionIsStaticShape (const NewtonCollision* const collision);

	// for the end user
	 void NewtonCollisionSetUserData (const NewtonCollision* const collision, void* const userData);
	 void* NewtonCollisionGetUserData (const NewtonCollision* const collision);
	
	 void NewtonCollisionSetUserID (const NewtonCollision* const collision, unsigned id);
	 unsigned NewtonCollisionGetUserID (const NewtonCollision* const collision);

	 void NewtonCollisionGetMaterial (const NewtonCollision* const collision, NewtonCollisionMaterial* const userData);
	 void NewtonCollisionSetMaterial (const NewtonCollision* const collision, const NewtonCollisionMaterial* const userData);

	 void* NewtonCollisionGetSubCollisionHandle (const NewtonCollision* const collision);
	 NewtonCollision* NewtonCollisionGetParentInstance (const NewtonCollision* const collision);

	 void NewtonCollisionSetMatrix (const NewtonCollision* const collision, const dFloat* const matrix);
	 void NewtonCollisionGetMatrix (const NewtonCollision* const collision, dFloat* const matrix);

	 void NewtonCollisionSetScale (const NewtonCollision* const collision, dFloat scaleX, dFloat scaleY, dFloat scaleZ);
	 void NewtonCollisionGetScale (const NewtonCollision* const collision, dFloat* const scaleX, dFloat* const scaleY, dFloat* const scaleZ);
	 void NewtonDestroyCollision (const NewtonCollision* const collision);

	 dFloat NewtonCollisionGetSkinThickness (const NewtonCollision* const collision);
	 void NewtonCollisionSetSkinThickness(const NewtonCollision* const collision, dFloat thickness);

	 int NewtonCollisionIntersectionTest (const NewtonWorld* const newtonWorld, 
		const NewtonCollision* const collisionA, const dFloat* const matrixA, 
		const NewtonCollision* const collisionB, const dFloat* const matrixB, int threadIndex);

	 int NewtonCollisionPointDistance (const NewtonWorld* const newtonWorld, const dFloat* const point,
		const NewtonCollision* const collision, const dFloat* const matrix, dFloat* const contact, dFloat* const normal, int threadIndex);

	 int NewtonCollisionClosestPoint (const NewtonWorld* const newtonWorld, 
		const NewtonCollision* const collisionA, const dFloat* const matrixA, 
		const NewtonCollision* const collisionB, const dFloat* const matrixB,
		dFloat* const contactA, dFloat* const contactB, dFloat* const normalAB, int threadIndex);

	 int NewtonCollisionCollide (const NewtonWorld* const newtonWorld, int maxSize,
		const NewtonCollision* const collisionA, const dFloat* const matrixA, 
		const NewtonCollision* const collisionB, const dFloat* const matrixB,
		dFloat* const contacts, dFloat* const normals, dFloat* const penetration, 
		dLong* const attributeA, dLong* const attributeB, int threadIndex);

	 int NewtonCollisionCollideContinue (const NewtonWorld* const newtonWorld, int maxSize, dFloat timestep, 
		const NewtonCollision* const collisionA, const dFloat* const matrixA, const dFloat* const velocA, const dFloat* omegaA, 
		const NewtonCollision* const collisionB, const dFloat* const matrixB, const dFloat* const velocB, const dFloat* const omegaB, 
		dFloat* const timeOfImpact, dFloat* const contacts, dFloat* const normals, dFloat* const penetration, 
		dLong* const attributeA, dLong* const attributeB, int threadIndex);

	 void NewtonCollisionSupportVertex (const NewtonCollision* const collision, const dFloat* const dir, dFloat* const vertex);
	 dFloat NewtonCollisionRayCast (const NewtonCollision* const collision, const dFloat* const p0, const dFloat* const p1, dFloat* const normal, dLong* const attribute);
	 void NewtonCollisionCalculateAABB (const NewtonCollision* const collision, const dFloat* const matrix, dFloat* const p0, dFloat* const p1);
	 void NewtonCollisionForEachPolygonDo (const NewtonCollision* const collision, const dFloat* const matrix, NewtonCollisionIterator callback, void* const userData);
	
	// **********************************************************************************************
	// 
	// collision aggregates, are a collision node on eh broad phase the serve as the root nod for a collection of rigid bodies
	// that shared the property of being in close proximity all the time, they are similar to compound collision by the group bodies instead of collision instances
	// These are good for speeding calculation calculation of rag doll, Vehicles or contractions of rigid bodied lined by joints.
	// also for example if you know that many the life time of a group of bodies like the object on a house of a building will be localize to the confide of the building
	// then warping the bodies under an aggregate will reduce collision calculation of almost an order of magnitude.
	//
	// **********************************************************************************************
	 void* NewtonCollisionAggregateCreate (NewtonWorld* const world); 	
	 void NewtonCollisionAggregateDestroy (void* const aggregate); 	
	 void NewtonCollisionAggregateAddBody (void* const aggregate, const NewtonBody* const body);
	 void NewtonCollisionAggregateRemoveBody (void* const aggregate, const NewtonBody* const body); 	

	 int NewtonCollisionAggregateGetSelfCollision (void* const aggregate);
	 void NewtonCollisionAggregateSetSelfCollision (void* const aggregate, int state);
	
	// **********************************************************************************************
	//
	// transforms utility functions
	//
	// **********************************************************************************************
	 void NewtonSetEulerAngle (const dFloat* const eulersAngles, dFloat* const matrix);
	 void NewtonGetEulerAngle (const dFloat* const matrix, dFloat* const eulersAngles0, dFloat* const eulersAngles1);
	 dFloat NewtonCalculateSpringDamperAcceleration (dFloat dt, dFloat ks, dFloat x, dFloat kd, dFloat s);

	// **********************************************************************************************
	//
	// body manipulation functions
	//
	// **********************************************************************************************
	 NewtonBody* NewtonCreateDynamicBody (const NewtonWorld* const newtonWorld, const NewtonCollision* const collision, const dFloat* const matrix);
	 NewtonBody* NewtonCreateKinematicBody (const NewtonWorld* const newtonWorld, const NewtonCollision* const collision, const dFloat* const matrix);
	 NewtonBody* NewtonCreateAsymetricDynamicBody(const NewtonWorld* const newtonWorld, const NewtonCollision* const collision, const dFloat* const matrix);

	 void NewtonDestroyBody(const NewtonBody* const body);

	 int NewtonBodyGetSimulationState(const NewtonBody* const body);
	 void NewtonBodySetSimulationState(const NewtonBody* const bodyPtr, const int state);

	 int NewtonBodyGetType (const NewtonBody* const body);
	 int NewtonBodyGetCollidable (const NewtonBody* const body);
	 void NewtonBodySetCollidable (const NewtonBody* const body, int collidableState);

	 void  NewtonBodyAddForce (const NewtonBody* const body, const dFloat* const force);
	 void  NewtonBodyAddTorque (const NewtonBody* const body, const dFloat* const torque);
	 void  NewtonBodyCalculateInverseDynamicsForce (const NewtonBody* const body, dFloat timestep, const dFloat* const desiredVeloc, dFloat* const forceOut);

	 void  NewtonBodySetCentreOfMass (const NewtonBody* const body, const dFloat* const com);
	 void  NewtonBodySetMassMatrix (const NewtonBody* const body, dFloat mass, dFloat Ixx, dFloat Iyy, dFloat Izz);
	 void  NewtonBodySetFullMassMatrix (const NewtonBody* const body, dFloat mass, const dFloat* const inertiaMatrix);

	 void  NewtonBodySetMassProperties (const NewtonBody* const body, dFloat mass, const NewtonCollision* const collision);
	 void  NewtonBodySetMatrix (const NewtonBody* const body, const dFloat* const matrix);
	 void  NewtonBodySetMatrixNoSleep (const NewtonBody* const body, const dFloat* const matrix);
	 void  NewtonBodySetMatrixRecursive (const NewtonBody* const body, const dFloat* const matrix);
	
	 void  NewtonBodySetMaterialGroupID (const NewtonBody* const body, int id);
	 void  NewtonBodySetContinuousCollisionMode (const NewtonBody* const body, unsigned state);
	 void  NewtonBodySetJointRecursiveCollision (const NewtonBody* const body, unsigned state);
	 void  NewtonBodySetOmega (const NewtonBody* const body, const dFloat* const omega);
	 void  NewtonBodySetOmegaNoSleep (const NewtonBody* const body, const dFloat* const omega);
	 void  NewtonBodySetVelocity (const NewtonBody* const body, const dFloat* const velocity);
	 void  NewtonBodySetVelocityNoSleep (const NewtonBody* const body, const dFloat* const velocity);
	 void  NewtonBodySetForce (const NewtonBody* const body, const dFloat* const force);
	 void  NewtonBodySetTorque (const NewtonBody* const body, const dFloat* const torque);
	
	 void  NewtonBodySetLinearDamping (const NewtonBody* const body, dFloat linearDamp);
	 void  NewtonBodySetAngularDamping (const NewtonBody* const body, const dFloat* const angularDamp);
	 void  NewtonBodySetCollision (const NewtonBody* const body, const NewtonCollision* const collision);
	 void  NewtonBodySetCollisionScale (const NewtonBody* const body, dFloat scaleX, dFloat  scaleY, dFloat scaleZ);

	 int  NewtonBodyGetSleepState (const NewtonBody* const body);
	 void NewtonBodySetSleepState (const NewtonBody* const body, int state);

	 int  NewtonBodyGetAutoSleep (const NewtonBody* const body);
	 void NewtonBodySetAutoSleep (const NewtonBody* const body, int state);

	 int  NewtonBodyGetFreezeState(const NewtonBody* const body);
	 void NewtonBodySetFreezeState (const NewtonBody* const body, int state);

	 void NewtonBodySetDestructorCallback (const NewtonBody* const body, NewtonBodyDestructor callback);
	 NewtonBodyDestructor NewtonBodyGetDestructorCallback (const NewtonBody* const body);

	 void  NewtonBodySetTransformCallback (const NewtonBody* const body, NewtonSetTransform callback);
	 NewtonSetTransform NewtonBodyGetTransformCallback (const NewtonBody* const body);
	
	 void  NewtonBodySetForceAndTorqueCallback (const NewtonBody* const body, NewtonApplyForceAndTorque callback);
	 NewtonApplyForceAndTorque NewtonBodyGetForceAndTorqueCallback (const NewtonBody* const body);

	 int NewtonBodyGetID (const NewtonBody* const body);

	 void  NewtonBodySetUserData (const NewtonBody* const body, void* const userData);
	 void* NewtonBodyGetUserData (const NewtonBody* const body);

	 NewtonWorld* NewtonBodyGetWorld (const NewtonBody* const body);
	 NewtonCollision* NewtonBodyGetCollision (const NewtonBody* const body);
	 int NewtonBodyGetMaterialGroupID (const NewtonBody* const body);

	 int NewtonBodyGetSerializedID(const NewtonBody* const body);
	 int NewtonBodyGetContinuousCollisionMode (const NewtonBody* const body);
	 int NewtonBodyGetJointRecursiveCollision (const NewtonBody* const body);

	 void NewtonBodyGetPosition(const NewtonBody* const body, dFloat* const pos);
	 void NewtonBodyGetMatrix(const NewtonBody* const body, dFloat* const matrix);
	 void NewtonBodyGetRotation(const NewtonBody* const body, dFloat* const rotation);
	 void NewtonBodyGetMass (const NewtonBody* const body, dFloat* mass, dFloat* const Ixx, dFloat* const Iyy, dFloat* const Izz);
	 void NewtonBodyGetInvMass(const NewtonBody* const body, dFloat* const invMass, dFloat* const invIxx, dFloat* const invIyy, dFloat* const invIzz);
	 void NewtonBodyGetInertiaMatrix(const NewtonBody* const body, dFloat* const inertiaMatrix);
	 void NewtonBodyGetInvInertiaMatrix(const NewtonBody* const body, dFloat* const invInertiaMatrix);
	 void NewtonBodyGetOmega(const NewtonBody* const body, dFloat* const vector);
	 void NewtonBodyGetVelocity(const NewtonBody* const body, dFloat* const vector);
	 void NewtonBodyGetAlpha(const NewtonBody* const body, dFloat* const vector);
	 void NewtonBodyGetAcceleration(const NewtonBody* const body, dFloat* const vector);
	 void NewtonBodyGetForce(const NewtonBody* const body, dFloat* const vector);
	 void NewtonBodyGetTorque(const NewtonBody* const body, dFloat* const vector);
	 void NewtonBodyGetCentreOfMass (const NewtonBody* const body, dFloat* const com);
	 void NewtonBodyGetPointVelocity (const NewtonBody* const body, const dFloat* const point, dFloat* const velocOut);

	 void NewtonBodyApplyImpulsePair (const NewtonBody* const body, dFloat* const linearImpulse, dFloat* const angularImpulse, dFloat timestep);
	 void NewtonBodyAddImpulse (const NewtonBody* const body, const dFloat* const pointDeltaVeloc, const dFloat* const pointPosit, dFloat timestep);
	 void NewtonBodyApplyImpulseArray (const NewtonBody* const body, int impuleCount, int strideInByte, const dFloat* const impulseArray, const dFloat* const pointArray, dFloat timestep);

	 void NewtonBodyIntegrateVelocity (const NewtonBody* const body, dFloat timestep);

	 dFloat NewtonBodyGetLinearDamping (const NewtonBody* const body);
	 void  NewtonBodyGetAngularDamping (const NewtonBody* const body, dFloat* const vector);
	 void  NewtonBodyGetAABB (const NewtonBody* const body, dFloat* const p0, dFloat* const p1);

	 NewtonJoint* NewtonBodyGetFirstJoint (const NewtonBody* const body);
	 NewtonJoint* NewtonBodyGetNextJoint (const NewtonBody* const body, const NewtonJoint* const joint);

	 NewtonJoint* NewtonBodyGetFirstContactJoint (const NewtonBody* const body);
	 NewtonJoint* NewtonBodyGetNextContactJoint (const NewtonBody* const body, const NewtonJoint* const contactJoint);
	 NewtonJoint* NewtonBodyFindContact (const NewtonBody* const body0, const NewtonBody* const body1);
	
	// **********************************************************************************************
	//
	// contact joints interface
	//
	// **********************************************************************************************
	 void* NewtonContactJointGetFirstContact (const NewtonJoint* const contactJoint);
	 void* NewtonContactJointGetNextContact (const NewtonJoint* const contactJoint, void* const contact);

	 int NewtonContactJointGetContactCount(const NewtonJoint* const contactJoint);
	 void NewtonContactJointRemoveContact(const NewtonJoint* const contactJoint, void* const contact); 

	 dFloat NewtonContactJointGetClosestDistance(const NewtonJoint* const contactJoint);
	 void NewtonContactJointResetSelfJointsCollision(const NewtonJoint* const contactJoint);

	 NewtonMaterial* NewtonContactGetMaterial (const void* const contact);

	 NewtonCollision* NewtonContactGetCollision0 (const void* const contact);	
	 NewtonCollision* NewtonContactGetCollision1 (const void* const contact);	

	 void* NewtonContactGetCollisionID0 (const void* const contact);	
	 void* NewtonContactGetCollisionID1 (const void* const contact);	
	
	
	// **********************************************************************************************
	//
	// Common joint functions
	//
	// **********************************************************************************************
	 void* NewtonJointGetUserData (const NewtonJoint* const joint);
	 void NewtonJointSetUserData (const NewtonJoint* const joint, void* const userData);

	 NewtonBody* NewtonJointGetBody0 (const NewtonJoint* const joint);
	 NewtonBody* NewtonJointGetBody1 (const NewtonJoint* const joint);

	 void NewtonJointGetInfo  (const NewtonJoint* const joint, NewtonJointRecord* const info);
	 int NewtonJointGetCollisionState (const NewtonJoint* const joint);
	 void NewtonJointSetCollisionState (const NewtonJoint* const joint, int state);

	 dFloat NewtonJointGetStiffness (const NewtonJoint* const joint);
	 void NewtonJointSetStiffness (const NewtonJoint* const joint, dFloat state);
	
	 void NewtonDestroyJoint(const NewtonWorld* const newtonWorld, const NewtonJoint* const joint);
	 void NewtonJointSetDestructor (const NewtonJoint* const joint, NewtonConstraintDestructor destructor);

	 int NewtonJointIsActive (const NewtonJoint* const joint);

	// **********************************************************************************************
	//
	// InverseDynamics Interface
	//
	// **********************************************************************************************
	 NewtonInverseDynamics* NewtonCreateInverseDynamics (const NewtonWorld* const newtonWorld);
	 void NewtonInverseDynamicsDestroy (NewtonInverseDynamics* const inverseDynamics);
	
	 void* NewtonInverseDynamicsGetRoot(NewtonInverseDynamics* const inverseDynamics);
	 NewtonBody* NewtonInverseDynamicsGetBody(NewtonInverseDynamics* const inverseDynamics, void* const node);
	 NewtonJoint* NewtonInverseDynamicsGetJoint(NewtonInverseDynamics* const inverseDynamics, void* const node);

	 NewtonJoint* NewtonInverseDynamicsCreateEffector(NewtonInverseDynamics* const inverseDynamics, void* const node, NewtonUserBilateralCallback callback);
	 void NewtonInverseDynamicsDestroyEffector(NewtonJoint* const effector);

	 void* NewtonInverseDynamicsAddRoot(NewtonInverseDynamics* const inverseDynamics, NewtonBody* const root);
	 void* NewtonInverseDynamicsAddChildNode(NewtonInverseDynamics* const inverseDynamics, void* const parentNode, NewtonJoint* const joint);
	 bool NewtonInverseDynamicsAddLoopJoint(NewtonInverseDynamics* const inverseDynamics, NewtonJoint* const joint);

	 void NewtonInverseDynamicsEndBuild (NewtonInverseDynamics* const inverseDynamics);

	 void NewtonInverseDynamicsUpdate (NewtonInverseDynamics* const inverseDynamics, dFloat timestep, int threadIndex);

	// **********************************************************************************************
	//
	// particle system interface (soft bodies, individual, pressure bodies and cloth)   
	//
	// **********************************************************************************************
	 NewtonCollision* NewtonCreateMassSpringDamperSystem (const NewtonWorld* const newtonWorld, int shapeID,
																	const dFloat* const points, int pointCount, int strideInBytes, const dFloat* const pointMass, 
																	const int* const links, int linksCount, const dFloat* const linksSpring, const dFloat* const linksDamper);

	 NewtonCollision* NewtonCreateDeformableSolid(const NewtonWorld* const newtonWorld, const NewtonMesh* const mesh, int shapeID);

	 int NewtonDeformableMeshGetParticleCount (const NewtonCollision* const deformableMesh); 
	 int NewtonDeformableMeshGetParticleStrideInBytes (const NewtonCollision* const deformableMesh); 
	 const dFloat* NewtonDeformableMeshGetParticleArray (const NewtonCollision* const deformableMesh); 

/*
	 NewtonCollision* NewtonCreateClothPatch (const NewtonWorld* const newtonWorld, NewtonMesh* const mesh, int shapeID, NewtonClothPatchMaterial* const structuralMaterial, NewtonClothPatchMaterial* const bendMaterial);
	 void NewtonDeformableMeshCreateClusters (NewtonCollision* const deformableMesh, int clusterCount, dFloat overlapingWidth);
	 void NewtonDeformableMeshSetDebugCallback (NewtonCollision* const deformableMesh, NewtonCollisionIterator callback);

	
	 void NewtonDeformableMeshGetParticlePosition (NewtonCollision* const deformableMesh, int particleIndex, dFloat* const posit);

	 void NewtonDeformableMeshBeginConfiguration (const NewtonCollision* const deformableMesh); 
	 void NewtonDeformableMeshUnconstraintParticle (NewtonCollision* const deformableMesh, int particleIndex);
	 void NewtonDeformableMeshConstraintParticle (NewtonCollision* const deformableMesh, int particleIndex, const dFloat* const posit, const NewtonBody* const body);
	 void NewtonDeformableMeshEndConfiguration (const NewtonCollision* const deformableMesh); 

//	 void NewtonDeformableMeshSetPlasticity (NewtonCollision* const deformableMesh, dFloat plasticity);
//	 void NewtonDeformableMeshSetStiffness (NewtonCollision* const deformableMesh, dFloat stiffness);
	 void NewtonDeformableMeshSetSkinThickness (NewtonCollision* const deformableMesh, dFloat skinThickness);

	 void NewtonDeformableMeshUpdateRenderNormals (const NewtonCollision* const deformableMesh); 
	 int NewtonDeformableMeshGetVertexCount (const NewtonCollision* const deformableMesh); 
	 void NewtonDeformableMeshGetVertexStreams (const NewtonCollision* const deformableMesh, int vertexStrideInByte, dFloat* const vertex, int normalStrideInByte, dFloat* const normal, int uvStrideInByte0, dFloat* const uv0);
	 NewtonDeformableMeshSegment* NewtonDeformableMeshGetFirstSegment (const NewtonCollision* const deformableMesh);
	 NewtonDeformableMeshSegment* NewtonDeformableMeshGetNextSegment (const NewtonCollision* const deformableMesh, const NewtonDeformableMeshSegment* const segment);

	 int NewtonDeformableMeshSegmentGetMaterialID (const NewtonCollision* const deformableMesh, const NewtonDeformableMeshSegment* const segment);
	 int NewtonDeformableMeshSegmentGetIndexCount (const NewtonCollision* const deformableMesh, const NewtonDeformableMeshSegment* const segment);
	 const int* NewtonDeformableMeshSegmentGetIndexList (const NewtonCollision* const deformableMesh, const NewtonDeformableMeshSegment* const segment);
*/
	// **********************************************************************************************
	//
	// Ball and Socket joint functions
	//
	// **********************************************************************************************
	 NewtonJoint* NewtonConstraintCreateBall (const NewtonWorld* const newtonWorld, const dFloat* pivotPoint, const NewtonBody* const childBody, const NewtonBody* const parentBody);
	 void NewtonBallSetUserCallback (const NewtonJoint* const ball, NewtonBallCallback callback);
	 void NewtonBallGetJointAngle (const NewtonJoint* const ball, dFloat* angle);
	 void NewtonBallGetJointOmega (const NewtonJoint* const ball, dFloat* omega);
	 void NewtonBallGetJointForce (const NewtonJoint* const ball, dFloat* const force);
	 void NewtonBallSetConeLimits (const NewtonJoint* const ball, const dFloat* pin, dFloat maxConeAngle, dFloat maxTwistAngle);

	// **********************************************************************************************
	//
	// Hinge joint functions
	//
	// **********************************************************************************************
	 NewtonJoint* NewtonConstraintCreateHinge (const NewtonWorld* const newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir, const NewtonBody* const childBody, const NewtonBody* const parentBody);
	 void NewtonHingeSetUserCallback (const NewtonJoint* const hinge, NewtonHingeCallback callback);
	 dFloat NewtonHingeGetJointAngle (const NewtonJoint* const hinge);
	 dFloat NewtonHingeGetJointOmega (const NewtonJoint* const hinge);
	 void NewtonHingeGetJointForce (const NewtonJoint* const hinge, dFloat* const force);
	 dFloat NewtonHingeCalculateStopAlpha (const NewtonJoint* const hinge, const NewtonHingeSliderUpdateDesc* const desc, dFloat angle);

	// **********************************************************************************************
	//
	// Slider joint functions
	//
	// **********************************************************************************************
	 NewtonJoint* NewtonConstraintCreateSlider (const NewtonWorld* const newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir, const NewtonBody* const childBody, const NewtonBody* const parentBody);
	 void NewtonSliderSetUserCallback (const NewtonJoint* const slider, NewtonSliderCallback callback);
	 dFloat NewtonSliderGetJointPosit (const NewtonJoint* slider);
	 dFloat NewtonSliderGetJointVeloc (const NewtonJoint* slider);
	 void NewtonSliderGetJointForce (const NewtonJoint* const slider, dFloat* const force);
	 dFloat NewtonSliderCalculateStopAccel (const NewtonJoint* const slider, const NewtonHingeSliderUpdateDesc* const desc, dFloat position);


	// **********************************************************************************************
	//
	// Corkscrew joint functions
	//
	// **********************************************************************************************
	 NewtonJoint* NewtonConstraintCreateCorkscrew (const NewtonWorld* const newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir, const NewtonBody* const childBody, const NewtonBody* const parentBody);
	 void NewtonCorkscrewSetUserCallback (const NewtonJoint* const corkscrew, NewtonCorkscrewCallback callback);
	 dFloat NewtonCorkscrewGetJointPosit (const NewtonJoint* const corkscrew);
	 dFloat NewtonCorkscrewGetJointAngle (const NewtonJoint* const corkscrew);
	 dFloat NewtonCorkscrewGetJointVeloc (const NewtonJoint* const corkscrew);
	 dFloat NewtonCorkscrewGetJointOmega (const NewtonJoint* const corkscrew);
	 void NewtonCorkscrewGetJointForce (const NewtonJoint* const corkscrew, dFloat* const force);
	 dFloat NewtonCorkscrewCalculateStopAlpha (const NewtonJoint* const corkscrew, const NewtonHingeSliderUpdateDesc* const desc, dFloat angle);
	 dFloat NewtonCorkscrewCalculateStopAccel (const NewtonJoint* const corkscrew, const NewtonHingeSliderUpdateDesc* const desc, dFloat position);


	// **********************************************************************************************
	//
	// Universal joint functions
	//
	// **********************************************************************************************
	 NewtonJoint* NewtonConstraintCreateUniversal (const NewtonWorld* const newtonWorld, const dFloat* pivotPoint, const dFloat* pinDir0, const dFloat* pinDir1, const NewtonBody* const childBody, const NewtonBody* const parentBody);
	 void NewtonUniversalSetUserCallback (const NewtonJoint* const universal, NewtonUniversalCallback callback);
	 dFloat NewtonUniversalGetJointAngle0 (const NewtonJoint* const universal);
	 dFloat NewtonUniversalGetJointAngle1 (const NewtonJoint* const universal);
	 dFloat NewtonUniversalGetJointOmega0 (const NewtonJoint* const universal);
	 dFloat NewtonUniversalGetJointOmega1 (const NewtonJoint* const universal);
	 void NewtonUniversalGetJointForce (const NewtonJoint* const universal, dFloat* const force);
	 dFloat NewtonUniversalCalculateStopAlpha0 (const NewtonJoint* const universal, const NewtonHingeSliderUpdateDesc* const desc, dFloat angle);
	 dFloat NewtonUniversalCalculateStopAlpha1 (const NewtonJoint* const universal, const NewtonHingeSliderUpdateDesc* const desc, dFloat angle);


	// **********************************************************************************************
	//
	// Up vector joint functions
	//
	// **********************************************************************************************
	 NewtonJoint* NewtonConstraintCreateUpVector (const NewtonWorld* const newtonWorld, const dFloat* pinDir, const NewtonBody* const body); 
	 void NewtonUpVectorGetPin (const NewtonJoint* const upVector, dFloat *pin);
	 void NewtonUpVectorSetPin (const NewtonJoint* const upVector, const dFloat *pin);


	// **********************************************************************************************
	//
	// User defined bilateral Joint
	//
	// **********************************************************************************************
	 NewtonJoint* NewtonConstraintCreateUserJoint (const NewtonWorld* const newtonWorld, int maxDOF, NewtonUserBilateralCallback callback, const NewtonBody* const childBody, const NewtonBody* const parentBody) ; 
	 int NewtonUserJointGetSolverModel(const NewtonJoint* const joint);
	 void NewtonUserJointSetSolverModel(const NewtonJoint* const joint, int model);
	 void NewtonUserJointSetFeedbackCollectorCallback (const NewtonJoint* const joint, NewtonUserBilateralCallback getFeedback);
	 void NewtonUserJointAddLinearRow (const NewtonJoint* const joint, const dFloat* const pivot0, const dFloat* const pivot1, const dFloat* const dir);
	 void NewtonUserJointAddAngularRow (const NewtonJoint* const joint, dFloat relativeAngle, const dFloat* const dir);
	 void NewtonUserJointAddGeneralRow (const NewtonJoint* const joint, const dFloat* const jacobian0, const dFloat* const jacobian1);
	 void NewtonUserJointSetRowMinimumFriction (const NewtonJoint* const joint, dFloat friction);
	 void NewtonUserJointSetRowMaximumFriction (const NewtonJoint* const joint, dFloat friction);
	 dFloat NewtonUserJointCalculateRowZeroAccelaration (const NewtonJoint* const joint);
	 dFloat NewtonUserJointGetRowAcceleration (const NewtonJoint* const joint);
	 void NewtonUserJointSetRowAsInverseDynamics (const NewtonJoint* const joint);
	 void NewtonUserJointSetRowAcceleration (const NewtonJoint* const joint, dFloat acceleration);
	 void NewtonUserJointSetRowSpringDamperAcceleration (const NewtonJoint* const joint, dFloat rowStiffness, dFloat spring, dFloat damper);
	 void NewtonUserJointSetRowStiffness (const NewtonJoint* const joint, dFloat stiffness);
	 int NewtonUserJoinRowsCount (const NewtonJoint* const joint);
	 void NewtonUserJointGetGeneralRow (const NewtonJoint* const joint, int index, dFloat* const jacobian0, dFloat* const jacobian1);
	 dFloat NewtonUserJointGetRowForce (const NewtonJoint* const joint, int row);

	// **********************************************************************************************
	//
	// Mesh joint functions
	//
	// **********************************************************************************************
	 NewtonMesh* NewtonMeshCreate(const NewtonWorld* const newtonWorld);
	 NewtonMesh* NewtonMeshCreateFromMesh(const NewtonMesh* const mesh);
	 NewtonMesh* NewtonMeshCreateFromCollision(const NewtonCollision* const collision);
	 NewtonMesh* NewtonMeshCreateTetrahedraIsoSurface(const NewtonMesh* const mesh);
	 NewtonMesh* NewtonMeshCreateConvexHull (const NewtonWorld* const newtonWorld, int pointCount, const dFloat* const vertexCloud, int strideInBytes, dFloat tolerance);
	 NewtonMesh* NewtonMeshCreateVoronoiConvexDecomposition (const NewtonWorld* const newtonWorld, int pointCount, const dFloat* const vertexCloud, int strideInBytes, int materialID, const dFloat* const textureMatrix);
	 NewtonMesh* NewtonMeshCreateFromSerialization (const NewtonWorld* const newtonWorld, NewtonDeserializeCallback deserializeFunction, void* const serializeHandle);
	 void NewtonMeshDestroy(const NewtonMesh* const mesh);

	 void NewtonMeshSerialize (const NewtonMesh* const mesh, NewtonSerializeCallback serializeFunction, void* const serializeHandle);
	 void NewtonMeshSaveOFF(const NewtonMesh* const mesh, const char* const filename);
	 NewtonMesh* NewtonMeshLoadOFF(const NewtonWorld* const newtonWorld, const char* const filename);
	 NewtonMesh* NewtonMeshLoadTetrahedraMesh(const NewtonWorld* const newtonWorld, const char* const filename);

	 void NewtonMeshApplyTransform (const NewtonMesh* const mesh, const dFloat* const matrix);
	 void NewtonMeshCalculateOOBB(const NewtonMesh* const mesh, dFloat* const matrix, dFloat* const x, dFloat* const y, dFloat* const z);

	 void NewtonMeshCalculateVertexNormals(const NewtonMesh* const mesh, dFloat angleInRadians);
	 void NewtonMeshApplySphericalMapping(const NewtonMesh* const mesh, int material, const dFloat* const aligmentMatrix);
	 void NewtonMeshApplyCylindricalMapping(const NewtonMesh* const mesh, int cylinderMaterial, int capMaterial, const dFloat* const aligmentMatrix);
	 void NewtonMeshApplyBoxMapping(const NewtonMesh* const mesh, int frontMaterial, int sideMaterial, int topMaterial, const dFloat* const aligmentMatrix);
	 void NewtonMeshApplyAngleBasedMapping(const NewtonMesh* const mesh, int material, NewtonReportProgress reportPrograssCallback, void* const reportPrgressUserData, dFloat* const aligmentMatrix);

	 void NewtonCreateTetrahedraLinearBlendSkinWeightsChannel(const NewtonMesh* const tetrahedraMesh, NewtonMesh* const skinMesh);
	
	 void NewtonMeshOptimize (const NewtonMesh* const mesh);
	 void NewtonMeshOptimizePoints (const NewtonMesh* const mesh);
	 void NewtonMeshOptimizeVertex (const NewtonMesh* const mesh);
	 int NewtonMeshIsOpenMesh (const NewtonMesh* const mesh);
	 void NewtonMeshFixTJoints (const NewtonMesh* const mesh);

	 void NewtonMeshPolygonize (const NewtonMesh* const mesh);
	 void NewtonMeshTriangulate (const NewtonMesh* const mesh);
	 NewtonMesh* NewtonMeshUnion (const NewtonMesh* const mesh, const NewtonMesh* const clipper, const dFloat* const clipperMatrix);
	 NewtonMesh* NewtonMeshDifference (const NewtonMesh* const mesh, const NewtonMesh* const clipper, const dFloat* const clipperMatrix);
	 NewtonMesh* NewtonMeshIntersection (const NewtonMesh* const mesh, const NewtonMesh* const clipper, const dFloat* const clipperMatrix);
	 void NewtonMeshClip (const NewtonMesh* const mesh, const NewtonMesh* const clipper, const dFloat* const clipperMatrix, NewtonMesh** const topMesh, NewtonMesh** const bottomMesh);

	 NewtonMesh* NewtonMeshConvexMeshIntersection (const NewtonMesh* const mesh, const NewtonMesh* const convexMesh);

	 NewtonMesh* NewtonMeshSimplify (const NewtonMesh* const mesh, int maxVertexCount, NewtonReportProgress reportPrograssCallback, void* const reportPrgressUserData);
	 NewtonMesh* NewtonMeshApproximateConvexDecomposition (const NewtonMesh* const mesh, dFloat maxConcavity, dFloat backFaceDistanceFactor, int maxCount, int maxVertexPerHull, NewtonReportProgress reportProgressCallback, void* const reportProgressUserData);

	 void NewtonRemoveUnusedVertices(const NewtonMesh* const mesh, int* const vertexRemapTable);

	 void NewtonMeshBeginBuild(const NewtonMesh* const mesh);
		 void NewtonMeshBeginFace(const NewtonMesh* const mesh);
			 void NewtonMeshAddPoint(const NewtonMesh* const mesh, dFloat64 x, dFloat64 y, dFloat64 z);
			 void NewtonMeshAddLayer(const NewtonMesh* const mesh, int layerIndex);
			 void NewtonMeshAddMaterial(const NewtonMesh* const mesh, int materialIndex);
			 void NewtonMeshAddNormal(const NewtonMesh* const mesh, dFloat x, dFloat y, dFloat z);
			 void NewtonMeshAddBinormal(const NewtonMesh* const mesh, dFloat x, dFloat y, dFloat z);
			 void NewtonMeshAddUV0(const NewtonMesh* const mesh, dFloat u, dFloat v);
			 void NewtonMeshAddUV1(const NewtonMesh* const mesh, dFloat u, dFloat v);
		 void NewtonMeshEndFace(const NewtonMesh* const mesh);
	 void NewtonMeshEndBuild(const NewtonMesh* const mesh);

	 void NewtonMeshClearVertexFormat (NewtonMeshVertexFormat* const format);
	 void NewtonMeshBuildFromVertexListIndexList (const NewtonMesh* const mesh, const NewtonMeshVertexFormat* const format);

	 int NewtonMeshGetPointCount (const NewtonMesh* const mesh); 
	 const int* NewtonMeshGetIndexToVertexMap(const NewtonMesh* const mesh);
	 int NewtonMeshGetVertexWeights(const NewtonMesh* const mesh, int vertexIndex, int* const weightIndex, dFloat* const weightFactor);

	 void NewtonMeshGetVertexDoubleChannel (const NewtonMesh* const mesh, int vertexStrideInByte, dFloat64* const outBuffer);
	 void NewtonMeshGetVertexChannel (const NewtonMesh* const mesh, int vertexStrideInByte, dFloat* const outBuffer);
	 void NewtonMeshGetNormalChannel (const NewtonMesh* const mesh, int vertexStrideInByte, dFloat* const outBuffer);
	 void NewtonMeshGetBinormalChannel (const NewtonMesh* const mesh, int vertexStrideInByte, dFloat* const outBuffer);
	 void NewtonMeshGetUV0Channel (const NewtonMesh* const mesh, int vertexStrideInByte, dFloat* const outBuffer);
	 void NewtonMeshGetUV1Channel (const NewtonMesh* const mesh, int vertexStrideInByte, dFloat* const outBuffer);
	 void NewtonMeshGetVertexColorChannel (const NewtonMesh* const mesh, int vertexStrideInByte, dFloat* const outBuffer);

	 int NewtonMeshHasNormalChannel(const NewtonMesh* const mesh);
	 int NewtonMeshHasBinormalChannel(const NewtonMesh* const mesh);
	 int NewtonMeshHasUV0Channel(const NewtonMesh* const mesh);
	 int NewtonMeshHasUV1Channel(const NewtonMesh* const mesh);
	 int NewtonMeshHasVertexColorChannel(const NewtonMesh* const mesh);

	 void* NewtonMeshBeginHandle (const NewtonMesh* const mesh); 
	 void NewtonMeshEndHandle (const NewtonMesh* const mesh, void* const handle); 
	 int NewtonMeshFirstMaterial (const NewtonMesh* const mesh, void* const handle); 
	 int NewtonMeshNextMaterial (const NewtonMesh* const mesh, void* const handle, int materialId); 
	 int NewtonMeshMaterialGetMaterial (const NewtonMesh* const mesh, void* const handle, int materialId); 
	 int NewtonMeshMaterialGetIndexCount (const NewtonMesh* const mesh, void* const handle, int materialId); 
	 void NewtonMeshMaterialGetIndexStream (const NewtonMesh* const mesh, void* const handle, int materialId, int* const index); 
	 void NewtonMeshMaterialGetIndexStreamShort (const NewtonMesh* const mesh, void* const handle, int materialId, short int* const index); 

	 NewtonMesh* NewtonMeshCreateFirstSingleSegment (const NewtonMesh* const mesh); 
	 NewtonMesh* NewtonMeshCreateNextSingleSegment (const NewtonMesh* const mesh, const NewtonMesh* const segment); 

	 NewtonMesh* NewtonMeshCreateFirstLayer (const NewtonMesh* const mesh); 
	 NewtonMesh* NewtonMeshCreateNextLayer (const NewtonMesh* const mesh, const NewtonMesh* const segment); 

	 int NewtonMeshGetTotalFaceCount (const NewtonMesh* const mesh); 
	 int NewtonMeshGetTotalIndexCount (const NewtonMesh* const mesh); 
	 void NewtonMeshGetFaces (const NewtonMesh* const mesh, int* const faceIndexCount, int* const faceMaterial, void** const faceIndices); 

	 int NewtonMeshGetVertexCount (const NewtonMesh* const mesh); 
	 int NewtonMeshGetVertexStrideInByte (const NewtonMesh* const mesh); 
	 const dFloat64* NewtonMeshGetVertexArray (const NewtonMesh* const mesh); 

	 void* NewtonMeshGetFirstVertex (const NewtonMesh* const mesh);
	 void* NewtonMeshGetNextVertex (const NewtonMesh* const mesh, const void* const vertex);
	 int NewtonMeshGetVertexIndex (const NewtonMesh* const mesh, const void* const vertex);

	 void* NewtonMeshGetFirstPoint (const NewtonMesh* const mesh);
	 void* NewtonMeshGetNextPoint (const NewtonMesh* const mesh, const void* const point);
	 int NewtonMeshGetPointIndex (const NewtonMesh* const mesh, const void* const point);
	 int NewtonMeshGetVertexIndexFromPoint (const NewtonMesh* const mesh, const void* const point);
	
	 void* NewtonMeshGetFirstEdge (const NewtonMesh* const mesh);
	 void* NewtonMeshGetNextEdge (const NewtonMesh* const mesh, const void* const edge);
	 void NewtonMeshGetEdgeIndices (const NewtonMesh* const mesh, const void* const edge, int* const v0, int* const v1);
	// void NewtonMeshGetEdgePointIndices (const NewtonMesh* const mesh, const void* const edge, int* const v0, int* const v1);

	 void* NewtonMeshGetFirstFace (const NewtonMesh* const mesh);
	 void* NewtonMeshGetNextFace (const NewtonMesh* const mesh, const void* const face);
	 int NewtonMeshIsFaceOpen (const NewtonMesh* const mesh, const void* const face);
	 int NewtonMeshGetFaceMaterial (const NewtonMesh* const mesh, const void* const face);
	 int NewtonMeshGetFaceIndexCount (const NewtonMesh* const mesh, const void* const face);
	 void NewtonMeshGetFaceIndices (const NewtonMesh* const mesh, const void* const face, int* const indices);
	 void NewtonMeshGetFacePointIndices (const NewtonMesh* const mesh, const void* const face, int* const indices);
	 void NewtonMeshCalculateFaceNormal (const NewtonMesh* const mesh, const void* const face, dFloat64* const normal);

	 void NewtonMeshSetFaceMaterial (const NewtonMesh* const mesh, const void* const face, int matId);

    // ---------------------------------------------------------------------------------------------------------------------------------
    // Vehicle related structures
    enum DifferentialType
    {
        m_4WD,
        m_RWD,
        m_FWD,
    };

    typedef struct BasciCarParameters
    {
        dFloat MASS;
        dFloat TIRE_MASS;
        dFloat STEER_ANGLE;
        dFloat BRAKE_TORQUE;
        dFloat COM_Y_OFFSET;
        dFloat TIRE_TOP_SPEED_KMH;
    
        dFloat IDLE_TORQUE;
        dFloat IDLE_TORQUE_RPM;
    
        dFloat PEAK_TORQUE;
        dFloat PEAK_TORQUE_RPM;
    
        dFloat PEAK_HP;
        dFloat PEAK_HP_RPM;
    
        dFloat REDLINE_TORQUE;
        dFloat REDLINE_TORQUE_RPM;
    
        dFloat GEAR_1;
        dFloat GEAR_2;
        dFloat GEAR_3;
        dFloat REVERSE_GEAR;
    
        dFloat SUSPENSION_LENGTH;
        dFloat SUSPENSION_SPRING;
        dFloat SUSPENSION_DAMPER;
        dFloat LATERAL_STIFFNESS;
        dFloat LONGITUDINAL_STIFFNESS;
        dFloat ALIGNING_MOMENT_TRAIL;
    
        int m_differentialType;
        dMatrix m_tireaLigment;
    };
    
]]

local helpers = {}
helpers.zeroMatrix = function()

    local dmat = ffi.new("dMatrix[1]")
    dmat[0].m_front.m_x = 0.0
    dmat[0].m_front.m_y = 0.0
    dmat[0].m_front.m_z = 0.0
    dmat[0].m_front.m_w = 0.0

    dmat[0].m_up.m_x = 0.0
    dmat[0].m_up.m_y = 0.0
    dmat[0].m_up.m_z = 0.0
    dmat[0].m_up.m_w = 0.0

    dmat[0].m_right.m_x = 0.0
    dmat[0].m_right.m_y = 0.0
    dmat[0].m_right.m_z = 0.0
    dmat[0].m_right.m_w = 0.0

    dmat[0].m_posit.m_x = 0.0
    dmat[0].m_posit.m_y = 0.0
    dmat[0].m_posit.m_z = 0.0
    dmat[0].m_posit.m_w = 0.0
    return dmat
end

helpers.identityMatrix = function()

    local dmat = ffi.new("dMatrix[1]")
    dmat[0].m_front.m_x = 1.0
    dmat[0].m_front.m_y = 0.0
    dmat[0].m_front.m_z = 0.0
    dmat[0].m_front.m_w = 0.0

    dmat[0].m_up.m_x = 0.0
    dmat[0].m_up.m_y = 1.0
    dmat[0].m_up.m_z = 0.0
    dmat[0].m_up.m_w = 0.0

    dmat[0].m_right.m_x = 0.0
    dmat[0].m_right.m_y = 0.0
    dmat[0].m_right.m_z = 1.0
    dmat[0].m_right.m_w = 0.0

    dmat[0].m_posit.m_x = 0.0
    dmat[0].m_posit.m_y = 0.0
    dmat[0].m_posit.m_z = 0.0
    dmat[0].m_posit.m_w = 1.0
    return dmat
end 

dVectorScale = function( dvec, scale )

	local nvec = ffi.new("dVector[1]")
	nvec[0].m_x = dvec[0].m_x * scale
	nvec[0].m_y = dvec[0].m_y * scale
	nvec[0].m_z = dvec[0].m_z * scale
	nvec[0].m_w = dvec[0].m_w * scale
	return nvec
end

dMatrixRotateVector = function( dm, v )

	local nvec = ffi.new("dVector[1]")
	nvec[0].m_x = v.m_x * dm.m_front.m_x + v.m_y * dm.m_up.m_x + v.m_z * dm.m_right.m_x
	nvec[0].m_y = v.m_x * dm.m_front.m_y + v.m_y * dm.m_up.m_y + v.m_z * dm.m_right.m_y
	nvec[0].m_z = v.m_x * dm.m_front.m_z + v.m_y * dm.m_up.m_z + v.m_z * dm.m_right.m_z
	nvec[0].m_w = 0.0
	return nvec
end

dVectorAdd = function (vec1, vec2)

	local nvec = ffi.new("dVector[1]")
	nvec[0].m_x = vec1.m_x + vec2.m_x
	nvec[0].m_y = vec1.m_y + vec2.m_y
	nvec[0].m_z = vec1.m_z + vec2.m_z
	nvec[0].m_w = vec1.m_w + vec2.m_w
	return nvec
end

dMatrixTransformVector = function ( dmat, vec )

	local res = dMatrixRotateVector( dmat[0], vec[0] )
	return dVectorAdd(dmat[0].m_posit, res[0])
end

return { newton, helpers }