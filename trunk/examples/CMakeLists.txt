ADD_SUBDIRECTORY( basicdemo )
ADD_SUBDIRECTORY( dice )
ADD_SUBDIRECTORY( multithreaded )

# Requires multithreaded, but that's not always available
IF( BULLET_BulletMultiThreaded_LIBRARY OR BULLET_BulletMultiThreaded_LIBRARY_debug )
    ADD_SUBDIRECTORY( solver )
ENDIF( BULLET_BulletMultiThreaded_LIBRARY OR BULLET_BulletMultiThreaded_LIBRARY_debug )

# Deprecated in Bullet 2.76 when Bullet removed ColladaConverter.
# Removed from osgBullet build post-v1.1.0.
# ADD_SUBDIRECTORY( colladaread )
