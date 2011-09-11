SET( CATEGORY Example )

ADD_SUBDIRECTORY( basicdemo )
ADD_SUBDIRECTORY( centerofmass )
ADD_SUBDIRECTORY( collision )
ADD_SUBDIRECTORY( dice )
ADD_SUBDIRECTORY( handphysics )
ADD_SUBDIRECTORY( hinge )
ADD_SUBDIRECTORY( multithreaded )

# Requires multithreaded, but that's not always available
IF( BULLET_BulletMultiThreaded_LIBRARY OR BULLET_BulletMultiThreaded_LIBRARY_debug )
    ADD_SUBDIRECTORY( solver )
ENDIF( BULLET_BulletMultiThreaded_LIBRARY OR BULLET_BulletMultiThreaded_LIBRARY_debug )
