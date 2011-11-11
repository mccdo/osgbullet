/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2009-2011 by Kenneth Mark Bryden
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

#ifndef __OSGBDYNAMICS_CONSTRAINTS_H__
#define __OSGBDYNAMICS_CONSTRAINTS_H__ 1


#include <osgbDynamics/Export.h>

#include <osg/Object>
#include <osg/Matrix>
#include <osg/Vec3>
#include <osg/Vec2>
#include <osg/ref_ptr>

#include <btBulletDynamicsCommon.h>

#include <list>
#include <string>



namespace osgbDynamics
{



/** \class CreationRecord CreationRecord.h <osgbDynamics/CreationRecord.h>
*/
class OSGBDYNAMICS_EXPORT Constraint : public osg::Object
{
public:
    Constraint();
    Constraint( const Constraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,Constraint);

    virtual btTypedConstraint* getConstraint() const
    {
        return( _constraint );
    }
    virtual btSliderConstraint* getAsBtSlider() const { return( NULL ); }

    virtual std::string getTypeName() const
    {
        return( "Constraint" );
    }

protected:
    virtual ~Constraint();

    btTypedConstraint* _constraint;
};
typedef std::list< osg::ref_ptr< Constraint > > ConstraintList;


/** \class CreationRecord CreationRecord.h <osgbDynamics/CreationRecord.h>
*/
class OSGBDYNAMICS_EXPORT SliderConstraint : public Constraint
{
public:
    SliderConstraint();
    SliderConstraint( btRigidBody* rbA, btRigidBody* rbB=NULL );
    SliderConstraint( btRigidBody* rbA, const osg::Matrix& rbAXform,
            btRigidBody* rbB, const osg::Matrix& rbBXform,
            const osg::Vec3& slideAxisInA, const osg::Vec2& slideLimit );
    SliderConstraint( const SliderConstraint& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY );
    META_Object(osgbDynamics,SliderConstraint);

    virtual btSliderConstraint* getAsBtSlider() const;

    virtual std::string getTypeName() const
    {
        return( "SliderConstraint" );
    }

    void setAXform( const osg::Matrix& rbAXform );
    osg::Matrix getAXform() const
    {
        return( _rbAXform );
    }

    void setBXform( const osg::Matrix& rbBXform );
    osg::Matrix getBXform() const
    {
        return( _rbBXform );
    }

    void setAxisInA( const osg::Vec3& axisInA );
    osg::Vec3 getAxisInA() const
    {
        return( _slideAxisInA );
    }

    void setLimit( const osg::Vec2& limit );
    osg::Vec2 getLimit() const
    {
        return( _slideLimit );
    }

protected:
    virtual ~SliderConstraint();

    void createConstraint();

    btRigidBody* _rbA;
    btRigidBody* _rbB;
    osg::Matrix _rbAXform;
    osg::Matrix _rbBXform;
    osg::Vec3 _slideAxisInA;
    osg::Vec2 _slideLimit;
};


// osgbDynamics
}


// __OSGBDYNAMICS_CONSTRAINTS_H__
#endif
