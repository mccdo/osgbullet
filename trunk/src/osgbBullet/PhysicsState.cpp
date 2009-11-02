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

#include <osg/Notify>
#include <osg/Object>
#include <osg/Group>

#include "osgbBullet/PhysicsState.h"
#include "osgbBullet/OSGToCollada.h"

#include "osgwTools/RefID.h"

#include <btBulletDynamicsCommon.h>

namespace osgbBullet
{


PhysicsData::PhysicsData()
  : _version( 1 ),
    _fileName( std::string( "" ) ),
    _cr( NULL ),
    _body( NULL )
{
}
PhysicsData::PhysicsData( const PhysicsData& rhs, osg::CopyOp copyop )
{
    (*this) = rhs;
}
PhysicsData::~PhysicsData()
{
}

PhysicsData&
PhysicsData::operator=( const PhysicsData& rhs )
{
    _version = rhs._version;
    _fileName = rhs._fileName;
    _cr = rhs._cr;
    _body = rhs._body;

    return( *this );
}


PhysicsState::PhysicsState()
{
}
PhysicsState::PhysicsState( const osgbBullet::PhysicsState& rhs, osg::CopyOp copyop )
{
}
PhysicsState::~PhysicsState()
{
}

void
PhysicsState::addPhysicsData( const osgwTools::RefID* id, PhysicsData* pd )
{
    if( _dataMap.find( id->str() ) != _dataMap.end() )
        osg::notify( osg::WARN ) << "Overwriting physics data for RefID " << id->str() << std::endl;

    _dataMap[ id->str() ] = pd;
}

void
PhysicsState::removePhysicsData( const osgwTools::RefID* id )
{
    DataMap::iterator it = _dataMap.find( id->str() );
    if( it == _dataMap.end() )
        osg::notify( osg::WARN ) << "Can't erase non-extant RefID (RefID::operator<<() TBD)." << std::endl;
    else
        _dataMap.erase( it );
}

const PhysicsState::DataMap&
PhysicsState::getDataMap() const
{
    return( _dataMap );
}


void
PhysicsState::addPhysicsData( const osgwTools::RefID* id, const btRigidBody* body )
{
    DataMap::iterator it = _dataMap.find( id->str() );
    if( it == _dataMap.end() )
    {
        osg::ref_ptr< PhysicsData > pd = new PhysicsData;
        pd->_body = const_cast< btRigidBody* >( body );
        _dataMap[ id->str() ] = pd.get();
    }
    else
    {
        it->second->_body = const_cast< btRigidBody* >( body );
    }
}

void
PhysicsState::addPhysicsData( const osgwTools::RefID* id, const osgbBullet::CreationRecord* cr )
{
    DataMap::iterator it = _dataMap.find( id->str() );
    if( it == _dataMap.end() )
    {
        osg::ref_ptr< PhysicsData > pd = new PhysicsData;
        pd->_cr = const_cast< CreationRecord* >( cr );
        _dataMap[ id->str() ] = pd.get();
    }
    else
    {
        it->second->_cr = const_cast< CreationRecord* >( cr );
    }
}

void
PhysicsState::addPhysicsData( const osgwTools::RefID* id, const std::string& fileName )
{
    DataMap::iterator it = _dataMap.find( id->str() );
    if( it == _dataMap.end() )
    {
        osg::ref_ptr< PhysicsData > pd = new PhysicsData;
        pd->_fileName = fileName;
        _dataMap[ id->str() ] = pd.get();
    }
    else
    {
        it->second->_fileName = fileName;
    }
}


//void
//PhysicsState::addPhysicsData( const osgwTools::RefID* id, const btConstraint& constraint )
//{
//}



// namespace osgbBullet
}
