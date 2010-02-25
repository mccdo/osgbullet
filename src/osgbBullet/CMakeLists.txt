SET( LIB_NAME osgbBullet )

ADD_DEFINITIONS(-DOSGBBULLET_LIBRARY)
SET( HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME} )

IF( USE_COLLADA )
    SET( BULLET_COLLADA_HEADERS 
	 ${HEADER_PATH}/ColladaUtils.h
	 ${HEADER_PATH}/OSGToCollada.h
    )    
    SET( BULLET_COLLADA_SOURCES 
	  OSGToCollada.cpp
	  ColladaUtils.cpp
    )

ENDIF( USE_COLLADA )


SET( LIB_PUBLIC_HEADERS
    ${HEADER_PATH}/BoundingCone.h
    ${HEADER_PATH}/BoundingCylinder.h
    ${HEADER_PATH}/Chart.h
    ${HEADER_PATH}/CollectVerticesVisitor.h
    ${HEADER_PATH}/CollisionShapes.h
    ${HEADER_PATH}/ComputeCylinderVisitor.h
    ${HEADER_PATH}/ComputeTriMeshVisitor.h
    ${HEADER_PATH}/CreationRecord.h
    ${HEADER_PATH}/Export.h
    ${HEADER_PATH}/GLDebugDrawer.h
    ${HEADER_PATH}/MotionState.h
    ${HEADER_PATH}/PhysicsState.h
    ${HEADER_PATH}/PhysicsThread.h
    ${HEADER_PATH}/RefCollisionShape.h
    ${HEADER_PATH}/RefCollisionObject.h
    ${HEADER_PATH}/RefRigidBody.h
    ${HEADER_PATH}/RigidBodyAnimation.h
    ${HEADER_PATH}/TripleBuffer.h
    ${HEADER_PATH}/Utils.h
    ${HEADER_PATH}/Version.h
    ${BULLET_COLLADA_HEADERS}
)

ADD_SHARED_LIBRARY_INTERNAL( ${LIB_NAME}
    ${OPENSCENEGRAPH_USER_DEFINED_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    BoundingCone.cpp
    BoundingCylinder.cpp
    Chart.cpp
    CollectVerticesVisitor.cpp
    CollisionShapes.cpp
    ComputeCylinderVisitor.cpp
    ComputeTriMeshVisitor.cpp
    CreationRecord.cpp
    GLDebugDrawer.cpp
    MotionState.cpp
    PhysicsState.cpp
    PhysicsThread.cpp
    RefCollisionObject.cpp
    RefCollisionShape.cpp
    RefRigidBody.cpp
    RigidBodyAnimation.cpp
    TripleBuffer.cpp
    Utils.cpp
    Version.cpp
    ${BULLET_COLLADA_SOURCES}
) 


TARGET_LINK_LIBRARIES(
    ${LIB_NAME}
    ${OSG_LIBRARIES}
    ${BULLET_LIBRARIES}
    ${OSGWORKS_LIBRARIES}
)

INCLUDE( ModuleInstall REQUIRED )
