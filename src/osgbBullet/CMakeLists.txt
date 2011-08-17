set( LIB_NAME osgbBullet )

if( BUILD_SHARED_LIBS )
    add_definitions( -DOSGBBULLET_LIBRARY )
endif()


set( HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME} )
set( LIB_PUBLIC_HEADERS
    ${HEADER_PATH}/OSGToCollada.h
    ${HEADER_PATH}/CreationRecord.h
    ${HEADER_PATH}/Export.h
    ${HEADER_PATH}/GroundPlane.h
    ${HEADER_PATH}/MotionState.h
    ${HEADER_PATH}/PhysicsState.h
    ${HEADER_PATH}/PhysicsThread.h
    ${HEADER_PATH}/RefRigidBody.h
    ${HEADER_PATH}/RigidBodyAnimation.h
    ${HEADER_PATH}/TripleBuffer.h
)

set( _osgBulletSourceFiles
    OSGToCollada.cpp
    CreationRecord.cpp
    GroundPlane.cpp
    MotionState.cpp
    PhysicsState.cpp
    PhysicsThread.cpp
    RefRigidBody.cpp
    RigidBodyAnimation.cpp
    TripleBuffer.cpp
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

    if( WIN32 )
        set( RELATIVE_LIB_PATH ../../lib/ )
    endif()
    # osgbBullet depends on osgbCollision
    target_link_libraries( ${LIB_NAME}
        optimized "osgbCollision"
        debug "${RELATIVE_LIB_PATH}osgbCollision${CMAKE_DEBUG_POSTFIX}"
    )
endif()

set_target_properties( ${LIB_NAME} PROPERTIES VERSION ${OSGBULLET_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES SOVERSION ${OSGBULLET_VERSION} )
set_target_properties( ${LIB_NAME} PROPERTIES PROJECT_LABEL "Lib ${LIB_NAME}" )

include( ModuleInstall REQUIRED )
