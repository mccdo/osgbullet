/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009 by Kenneth Mark Bryden
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *************** <auto-copyright.pl END do not edit this line> ***************/

#ifndef __OSGBBULLET_OSG_TO_COLLADA_H__
#define __OSGBBULLET_OSG_TO_COLLADA_H__ 1


#include <osgbBullet/CollisionShapes.h>
#include <osgbBullet/CreationRecord.h>
#include <osgbBullet/Export.h>

#include <btBulletDynamicsCommon.h>

#include <string>



namespace osgbBullet {



// This is a support class for the osgbpp application, so that
// an app can programmatically do the same thing as osgbpp:
// Namely, convert an input scene graph into COLLADA physics data.
class OSGBBULLET_EXPORT OSGToCollada
{
public:
    // NOTR: loadedModel is _not_ const. This _will_ alter your scene graph.
    // It flattens transforms.
    // It removes loaded ProxyNodes (with the Optimizer).
    //
    // 'overall' and 'nodeName' work together to determine how the Bullet collision
    // shape is created (which is used to create the rigid body that is written to
    // the Collada file):
    //   overall  nodeName   Results
    //    true     empty     A single collision shape is created based on the bounding volume of the entire 'sg' scene graph.
    //    true    non-empty  'sg' is searched until a node with name 'nodeName' is found, then a single collision shape is created based on the bounding volume of the subgraph rooted at that node.
    //    false    empty     Collision shapes are created for all Geodes in 'sg' and assembled into a compound collision shape.
    //    false   non-empty  'sg' is searched until a node with name 'nodeName' is found, then collision shapes are created for all Geodes below that node and assembled into a compound collision shape.
    // Typical expected usage is 'overall' false and 'nodeName' empty.
    OSGToCollada();
    OSGToCollada( CreationRecord& cr );
    ~OSGToCollada();

    void init();

    bool convert( const std::string& outputFileName = std::string( "" ) );


    btRigidBody* getRigidBody();
    CreationRecord* getOrCreateCreationRecord();
    const CreationRecord* getOrCreateCreationRecord() const;


    void setSceneGraph( osg::Node* sg );
    osg::Node* getSceneGraph() const;

    // Specifying the center of mass automatically disables auto-compute center of mass.
    void setCenterOfMass( const osg::Vec3& com );
    const osg::Vec3& getCenterOfMass() const;
    void setAutoComputeCenterOfMass( bool compute );
    bool getAutoComputeCenterOfMass() const;

    void setShapeType( const BroadphaseNativeTypes shapeType );
    BroadphaseNativeTypes getShapeType() const;

    void setMass( float mass );
    float getMass() const;

    void setOverall( bool overall );
    bool getOverall() const;

    void setNodeName( const std::string& nodeName );
    const std::string& getNodeName() const;

    // for cylinder alignment only, ignored otherwise.
    void setAxis( osgbBullet::AXIS axis );
    osgbBullet::AXIS getAxis() const;

    // OSGToCollada already transforms all vertices by the inverse COM and the
    // accumulated model transformations in the specified
    // scene graph (setSceneGraph()) before computing the collision shape. 
    // If the subgraph parent transformation contains a local scale,
    // specify it here, and the resulting collision shape CSprime will be
    // computed as follows:
    //    CSprime = vertexData M -C S 
    // ... the vertex data is transformed by subgrph model transforms, then offset
    // by the inverse COM, then scaled.
    void setScale( const osg::Vec3& scale );
    osg::Vec3 getScale() const;

protected:
    btDynamicsWorld* initPhysics();
    btRigidBody* createRigidBody( btScalar mass,
        btCollisionShape* shape, const osg::Vec3& centerOfMass, osg::Node* node );


    btRigidBody* _rigidBody;
    mutable osg::ref_ptr< CreationRecord > _cr;
};

}

#endif
