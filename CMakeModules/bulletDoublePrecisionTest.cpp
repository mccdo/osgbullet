
#include <BulletCollision/btBulletCollisionCommon.h>

int main()
{
    btVector3 vec( 0., 1., 0. );
    btStaticPlaneShape* bsps = new btStaticPlaneShape( vec, 0. );

    return( 0 );
}
