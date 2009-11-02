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

#ifndef OSGBBULLET_COMPUTECYLINDERVISITOR
#define OSGBBULLET_COMPUTECYLINDERVISITOR    1

#include <osg/NodeVisitor>
#include <osg/Version>

#include <osgbBullet/BoundingCylinder.h>

namespace osgbBullet {

/* TBD Consider using OSG localtoworld method instead of keeping a matrix stack. */
class ComputeCylinderVisitor
    : public osg::NodeVisitor
{
public:
    ComputeCylinderVisitor( osg::NodeVisitor::TraversalMode traversalMode = TRAVERSE_ALL_CHILDREN );

#if( ( OPENSCENEGRAPH_MAJOR_VERSION >= 2) && (OPENSCENEGRAPH_MINOR_VERSION >= 8) )
    META_NodeVisitor(osgbBullet,ComputeCylinderVisitor)
#endif

    virtual void reset();


    virtual void setAxis( const osg::Vec3 a )
    {
        axis = a;
        axis.normalize();
        bc.setAxis( axis );
    }

    BoundingCylinder & getBoundingCylinder()
    {
        return( bc );
    }

    void apply( osg::Transform & transform );
    void apply( osg::Geode & geode );


    inline void pushMatrix( osg::Matrix & matrix )
    {
        stack.push_back( matrix );
    }

    inline void popMatrix()
    {
        stack.pop_back();
    }

    void applyDrawable( osg::Drawable * drawable );

protected:
    typedef std::vector< osg::Matrix >   MatrixStack;

    MatrixStack stack;
    BoundingCylinder bc;
    osg::Vec3 axis;
};

}

#endif
