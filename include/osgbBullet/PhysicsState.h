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

#ifndef __OSGBBULLET_PHYSICS_STATE_H__
#define __OSGBBULLET_PHYSICS_STATE_H__ 1

#include <osg/Object>
#include <osg/Group>

#include "osgbBullet/Export.h"
#include "osgbBullet/OSGToCollada.h"

#include "osgwTools/RefID.h"

#include <btBulletDynamicsCommon.h>


namespace osgbBullet {


class OSGBBULLET_EXPORT PhysicsData : public osg::Object
{
public:
    PhysicsData();
    PhysicsData( const PhysicsData& rhs, osg::CopyOp copyop=osg::CopyOp::SHALLOW_COPY );

    PhysicsData& operator=( const PhysicsData& rhs );

    META_Object(osgbBullet,PhysicsData);

    unsigned int _version;

    std::string _fileName;
    osg::ref_ptr< osgbBullet::CreationRecord > _cr;
    btRigidBody* _body;

    // For save / restore use only
    osg::Matrix _osgTransform;
    osg::Matrix _bodyWorldTransform;
    osg::Vec3 _linearVelocity;
    osg::Vec3 _angularVelocity;

protected:
    ~PhysicsData();
};

class OSGBBULLET_EXPORT PhysicsState : public osg::Object
{
public:
    PhysicsState();
    PhysicsState( const osgbBullet::PhysicsState& rhs, osg::CopyOp copyop=osg::CopyOp::SHALLOW_COPY );
    ~PhysicsState();

    META_Object(osgbBullet,PhysicsState);

    void addPhysicsData( const osgwTools::RefID* id, PhysicsData* pd );
    void removePhysicsData( const osgwTools::RefID* id );

    typedef std::map< std::string, osg::ref_ptr< PhysicsData > > DataMap;
    const DataMap& getDataMap() const;

    void addPhysicsData( const osgwTools::RefID* id, const btRigidBody* body );
    void addPhysicsData( const osgwTools::RefID* id, const osgbBullet::CreationRecord* cr );
    void addPhysicsData( const osgwTools::RefID* id, const std::string& fileName );

    //void addPhysicsData( const osgwTools::RefID* id, const btConstraint& constraint );

protected:
    DataMap _dataMap;
};

// namespace osgbBullet
}

// __OSGBBULLET_PHYSICS_STATE_H__
#endif
