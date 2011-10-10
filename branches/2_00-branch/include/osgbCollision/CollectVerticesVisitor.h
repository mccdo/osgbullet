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

#ifndef __OSGBCOLLISION_COLLECT_VERTICES_VISITOR_H__
#define __OSGBCOLLISION_COLLECT_VERTICES_VISITOR_H__ 1

#include <osgbCollision/Export.h>
#include <osg/NodeVisitor>
#include <osg/Array>
#include <osgwTools/Version.h>


namespace osgbCollision
{


/** \class CollectVerticesVisitor CollectVerticesVisitor.h <osgbCollision/CollectVerticesVisitor.h>
\brief A NodeVisitor to collect a set of transformed vertices.

*/
class OSGBCOLLISION_EXPORT CollectVerticesVisitor : public osg::NodeVisitor
{
public:
    CollectVerticesVisitor( osg::NodeVisitor::TraversalMode traversalMode = osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );

#if( OSGWORKS_OSG_VERSION >= 20800 )
    META_NodeVisitor(osgbCollision,CollectVerticesVisitor);
#endif

    virtual void reset();

    osg::Vec3Array* getVertices()
    {
        return( verts_.get() );
    }

    void apply( osg::Geode& geode );

protected:
    void applyDrawable( osg::Drawable* drawable );

    osg::ref_ptr< osg::Vec3Array > verts_;
};


// osgbCollision
}


// __OSGBCOLLISION_COLLECT_VERTICES_VISITOR_H__
#endif
