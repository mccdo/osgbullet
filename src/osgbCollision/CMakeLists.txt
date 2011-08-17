set( LIB_NAME osgbCollision )

if( BUILD_SHARED_LIBS )
    add_definitions( -DOSGBBULLET_LIBRARY )
endif()


set( HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME} )
set( LIB_PUBLIC_HEADERS
    ${HEADER_PATH}/BoundingCone.h
    ${HEADER_PATH}/BoundingCylinder.h
    ${HEADER_PATH}/Chart.h
    ${HEADER_PATH}/CollisionShapes.h
    ${HEADER_PATH}/ComputeCylinderVisitor.h
    ${HEADER_PATH}/CollectVerticesVisitor.h
    ${HEADER_PATH}/ComputeTriMeshVisitor.h
    ${HEADER_PATH}/GLDebugDrawer.h
    ${HEADER_PATH}/RefCollisionObject.h
    ${HEADER_PATH}/RefCollisionShape.h
    ${HEADER_PATH}/Utils.h
    ${HEADER_PATH}/Version.h
    ${HEADER_PATH}/Export.h
)

set( _osgBulletSourceFiles
    BoundingCone.cpp
    BoundingCylinder.cpp
    Chart.cpp
    CollisionShapes.cpp
    ComputeCylinderVisitor.cpp
    CollectVerticesVisitor.cpp
    ComputeTriMeshVisitor.cpp
    GLDebugDrawer.cpp
    RefCollisionObject.cpp
    RefCollisionShape.cpp
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
        ${BULLET_COLLISION_LIBRARY}
        ${BULLET_MATH_LIBRARY}
    )
endif()

set_target_properties( ${LIB_NAME} PROPERTIES VERSION ${OSGBULLET_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES SOVERSION ${OSGBULLET_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES PROJECT_LABEL "Lib ${LIB_NAME}" )

include( ModuleInstall REQUIRED )
