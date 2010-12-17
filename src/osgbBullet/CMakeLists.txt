set( LIB_NAME osgbBullet )

if( BUILD_SHARED_LIBS )
    add_definitions( -DOSGBBULLET_LIBRARY )
endif()


set( HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME} )
set( LIB_PUBLIC_HEADERS
    ${HEADER_PATH}/BoundingCone.h
    ${HEADER_PATH}/BoundingCylinder.h
    ${HEADER_PATH}/Chart.h
    ${HEADER_PATH}/OSGToCollada.h
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
)

set( _osgBulletSourceFiles
    BoundingCone.cpp
    BoundingCylinder.cpp
    Chart.cpp
    OSGToCollada.cpp
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
)

add_library( ${LIB_NAME}
    ${LIB_PUBLIC_HEADERS}
    ${_osgBulletSourceFiles}
)
if( BUILD_SHARED_LIBS )
    target_link_libraries( ${LIB_NAME}
        ${OSGWORKS_LIBRARIES}
        ${OSG_LIBRARIES}
        ${BULLET_LIBRARIES}
    )
endif()

set_target_properties( ${LIB_NAME} PROPERTIES VERSION ${OSGBULLET_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES SOVERSION ${OSGBULLET_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES PROJECT_LABEL "Lib ${LIB_NAME}" )

include( ModuleInstall REQUIRED )
