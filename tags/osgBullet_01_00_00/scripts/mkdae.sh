#!/bin/bash

# This batch file runs osgbpp to create multiple DAE
# files for the tetra.osg and cow.osg models. You can
# load these files with the colladaread example.
# For example:
#   colladaread cow.osg
# then press 0, 1, or 2 to load cow0.dae, cow1.dae,
# or cow2.dae respectively.
# Or:
#   colladaread tetra.osg
# and hit the 0 or 1 key to load either of the two files.
#
# After the files are loaded and the physics data extracted,
# the data is added to the Bullet simulation and the cow or
# tetrahedron should fall to the ground.

# Use one of these two lines to control whether osgbpp does its post-generatrion display of the physics data.
# export DISPLAY=--display
export DISPLAY=

osgbpp ${DISPLAY} --box --overall -o tetra0.dae tetra.osg
osgbpp ${DISPLAY} --convexHull --overall -o tetra1.dae tetra.osg

osgbpp ${DISPLAY} --box --overall -o offcube0.dae offcube.osg
osgbpp ${DISPLAY} --convexHull --overall -o offcube1.dae offcube.osg

osgbpp ${DISPLAY} --box --overall -o concave0.dae concave.osg
osgbpp ${DISPLAY} --box -o concave1.dae concave.osg

osgbpp ${DISPLAY} --box --overall -o cow0.dae cow.osg
osgbpp ${DISPLAY} --convexHull --simplify .2 --overall -o cow1.dae cow.osg
osgbpp ${DISPLAY} --convexHull --overall -o cow2.dae cow.osg
