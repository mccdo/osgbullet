ADD_SUBDIRECTORY( basicdemo )
ADD_SUBDIRECTORY( dice )
ADD_SUBDIRECTORY( colladaread )
ADD_SUBDIRECTORY( multithreaded )

# Requires multithreaded, but that's not always available
IF( BULLET_BulletMultiThreaded_LIBRARY OR BULLET_BulletMultiThreaded_LIBRARY_debug AND USE_COLLADA)
    ADD_SUBDIRECTORY( solver )
ENDIF( BULLET_BulletMultiThreaded_LIBRARY OR BULLET_BulletMultiThreaded_LIBRARY_debug AND USE_COLLADA )
