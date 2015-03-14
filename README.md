# osgbullet
Automatically exported from code.google.com/p/osgbullet

osgBullet
osgBullet is a set of software tools for applications that use both OpenSceneGraph (OSG) and Bullet. The osgBullet library is the result of collaborative work between Paul Martz (Skew Matrix Software), Ames Lab, and ARDEC. It's used as part of the VE-Suite project. osgBullet is open source and available under the GNU LGPL v2.1 software license.

Documentation
Online Doxygen documentation.

Features
Applications
osgbpp: The osgBullet physics previewer. Loads a model file, creates a Bullet rigid body to match the geometry, and drops it on a surface. View a graphical representation of the collision shape with the --debug option. Use the --help option for more info.

Libraries
The osgbCollision library Includes the following features:

Supports using Bullet for collision detection, with no dependency on libBulletDynamics.
Utility functions for converting between Bullet collision shapes and OSG scene graphs (create a scene graph to render a collision shape, for example, or create a collision shape from a scene graph).
A NodeVisitor to compute collision shapes per Geode and assemble them into a single btCompoundShape.
Utility functions for converting between Bullet and OSG matrix and vector data types.
A template class to incorporate Bullet objects as OSG Objects to support reference counting and association as UserData. Allows you to reference count a btRigidBody, for example, or store a btCollisionShape as UserData.
The osgbDynamics library Includes the following features:

Supports interleaved/serial physics step and render, and a separate physics simulation thread. Threaded physics simulation features an efficient triple buffering mechanism for shared transform data.
A MotionState class to keep your OSG subgraph visual representation in sync with the Bullet rigid body / collision shape. Bullet effectively owns the OSG transformation matrix, and the MotionState transparently accounts for center of mass and scaling.
Convenience routines to create rigid bodies from scene graphs, which handles all of the details of creating a MotionState, setting the initial transformation, handling center of mass offset, and creating scaled collision shapes.
A GLDebugDrawer debugging aid class that renders information regarding Bullet collision shapes and intersection points, both within the OSG scene and graphically as a HUD.
Examples
The project contains a small collection of example programs to demonstrate use of many of osgBullet's features.

Tests
The project contains several test programs to ensure correct functionality.

Building, Running, and More
Lots of additional information is available on the wiki.

Support
Skew Matrix Software provides full osgBullet development and consulting services. For support requests beyond issuing issues through Google Code, please contact Skew Matrix Software LLC.

Contribute
All contributions are welcome and will be considered for inclusion in the project. Please contribute any enhancements or bug fixes by opening an issue and use the Contribution from user issue template. Create and attach a compressed patch file containing your changes.
