IF(WIN32)
    SET(CMAKE_DEBUG_POSTFIX d)
ENDIF(WIN32)

MACRO( ADD_OSGPLUGIN TRGTNAME )
    if( WIN32 )
        set( RELATIVE_LIB_PATH ../../../lib/ )
    endif()

    if( BUILD_SHARED_LIBS )
        add_library( ${TRGTNAME} MODULE ${ARGN} )

        link_internal( ${TRGTNAME}
            osgbBullet
            osgbCollision
        )
        target_link_libraries( ${TRGTNAME}
            ${OSGWORKS_LIBRARIES}
            ${OSG_LIBRARIES}
        )
    else()
        add_library( ${TRGTNAME} STATIC ${ARGN} )
    endif()

    IF( WIN32 )
        SET_TARGET_PROPERTIES( ${TRGTNAME} PROPERTIES DEBUG_POSTFIX d )
    ENDIF( WIN32 )
    SET_TARGET_PROPERTIES( ${TRGTNAME} PROPERTIES PREFIX "" )
    SET_TARGET_PROPERTIES( ${TRGTNAME} PROPERTIES PROJECT_LABEL "Plugin ${TRGTNAME}" )
ENDMACRO( ADD_OSGPLUGIN TRGTNAME )


macro( _osgBulletMakeDynamicsExe _exeName )
    set( _osgBulletLibs
        "osgbBullet;osgbCollision"
    )
    set( _bulletLibs
        "${BULLET_LIBRARIES}"
    )
    _osgBulletMakeExeInternal( ${_exeName} "${_osgBulletLibs}" "${_bulletLibs}" ${ARGN} )
endmacro()

macro( _osgBulletMakeCollisionExe _exeName )
    set( _osgBulletLibs
        "osgbCollision"
    )
    set( _bulletLibs
        "${BULLET_COLLISION_LIBRARY};${BULLET_MATH_LIBRARY}"
    )
    _osgBulletMakeExeInternal( ${_exeName} "${_osgBulletLibs}" "${_bulletLibs}" ${ARGN} )
endmacro()

macro( _osgBulletMakeExeInternal _exeName _osgBulletLibs _bulletLibs )
    add_executable( ${_exeName} ${ARGN} )
    if( WIN32 )
        set_target_properties( ${_exeName} PROPERTIES DEBUG_POSTFIX d )
        set( RELATIVE_LIB_PATH ../../lib/ )
    endif()

    link_internal( ${_exeName}
        ${_osgBulletLibs}
    )
    target_link_libraries( ${_exeName}
        ${OSGWORKS_LIBRARIES}
        ${OSG_LIBRARIES}
        ${_bulletLibs}
    )
    if( ${CATEGORY} STREQUAL "App" )
        install(
            TARGETS ${_exeName}
            RUNTIME DESTINATION bin COMPONENT libosgbbullet
        )
    else()
        install(
            TARGETS ${_exeName}
            RUNTIME DESTINATION share/${CMAKE_PROJECT_NAME}/bin COMPONENT libosgbbullet
        )
    endif()
    set_target_properties( ${_exeName} PROPERTIES PROJECT_LABEL "${CATEGORY} ${_exeName}" )
endmacro()


macro( link_internal TRGTNAME )
    foreach( LINKLIB ${ARGN} )
        target_link_libraries( ${TRGTNAME} optimized "${LINKLIB}" debug "${RELATIVE_LIB_PATH}${LINKLIB}${CMAKE_DEBUG_POSTFIX}" )
    endforeach()
endmacro()
