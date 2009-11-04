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

#ifndef OSGBBULLET_COMPUTETRIMESHVISITOR
#define OSGBBULLET_COMPUTETRIMESHVISITOR    1

#include <osg/NodeVisitor>
#include <osg/Array>
#include <osg/Version>

namespace osgbBullet {

/* TBD Consider using OSG localtoworld method instead of keeping a matrix stack. */
class ComputeTriMeshVisitor
    : public osg::NodeVisitor
{
public:
    ComputeTriMeshVisitor( osg::NodeVisitor::TraversalMode traversalMode = TRAVERSE_ALL_CHILDREN );

#if( ( OPENSCENEGRAPH_MAJOR_VERSION >= 2) && (OPENSCENEGRAPH_MINOR_VERSION >= 8) )
    META_NodeVisitor(osgbBullet,ComputeTriMeshVisitor)
#endif

    virtual void reset();


    osg::Vec3Array * getTriMesh()
    {
        return( mesh.get() );
    }

    void apply( osg::Node & node );


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
    osg::ref_ptr< osg::Vec3Array > mesh;
};

}

#endif
