SET( LIB_NAME osgbBullet )

ADD_DEFINITIONS(-DOSGBBULLET_LIBRARY)
SET( HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME} )

SET( LIB_PUBLIC_HEADERS
    ${HEADER_PATH}/BoundingCone.h
    ${HEADER_PATH}/BoundingCylinder.h
    ${HEADER_PATH}/Chart.h
    ${HEADER_PATH}/ColladaUtils.h
    ${HEADER_PATH}/CollectVerticesVisitor.h
    ${HEADER_PATH}/CollisionShapes.h
    ${HEADER_PATH}/ComputeCylinderVisitor.h
    ${HEADER_PATH}/ComputeTriMeshVisitor.h
    ${HEADER_PATH}/CreationRecord.h
    ${HEADER_PATH}/Export.h
    ${HEADER_PATH}/GLDebugDrawer.h
    ${HEADER_PATH}/MotionState.h
    ${HEADER_PATH}/OSGToCollada.h
    ${HEADER_PATH}/PhysicsState.h
    ${HEADER_PATH}/PhysicsThread.h
    ${HEADER_PATH}/RefCollisionShape.h
    ${HEADER_PATH}/RefRigidBody.h
    ${HEADER_PATH}/RigidBodyAnimation.h
    ${HEADER_PATH}/TripleBuffer.h
    ${HEADER_PATH}/Utils.h
    ${HEADER_PATH}/Version.h
)

ADD_SHARED_LIBRARY_INTERNAL( ${LIB_NAME}
    ${OPENSCENEGRAPH_USER_DEFINED_DYNAMIC_OR_STATIC}
    ${LIB_PUBLIC_HEADERS}
    BoundingCone.cpp
    BoundingCylinder.cpp
    Chart.cpp
    ColladaUtils.cpp
    CollectVerticesVisitor.cpp
    CollisionShapes.cpp
    ComputeCylinderVisitor.cpp
    ComputeTriMeshVisitor.cpp
    CreationRecord.cpp
    GLDebugDrawer.cpp
    MotionState.cpp
    OSGToCollada.cpp
    PhysicsState.cpp
    PhysicsThread.cpp
    RefCollisionShape.cpp
    RefRigidBody.cpp
    RigidBodyAnimation.cpp
    TripleBuffer.cpp
    Utils.cpp
    Version.cpp
)

TARGET_LINK_LIBRARIES(
    ${LIB_NAME}
    ${OSG_LIBRARIES}
    ${BULLET_LIBRARIES}
    ${OSGWORKS_LIBRARIES}
)

INCLUDE( ModuleInstall REQUIRED )
