#/bin/sh

echo
echo Turn-on tests. Does it work at all?
echo Run BasicDemo, RobotArm, fountain.
echo
BasicDemo
robot


echo
echo
echo
echo Decimation tests.
echo First, test the simpifier.
echo
osgbpp cow.osg -o out.dae --display --convexTM --overall
osgbpp cow.osg -o out.dae --display --convexTM --overall --simplify .2
echo
echo Decimation tests.
echo Test the decimator
echo
osgbpp cow.osg -o out.dae --display --convexTM --overall --decPercent .2
echo
echo Decimation tests.
echo Test the Vertex Aggregator.
echo
osgbpp cow.osg -o out.dae --display --convexTM --overall --aggMaxVerts 50
echo
echo Decimation tests, this time with the pliers.
echo First, test the simpifier.
echo
osgbpp pliers-big.osg -o out.dae --display --convexTM --overall
osgbpp pliers-big.osg -o out.dae --display --convexTM --overall --simplify .2
echo
echo Decimation tests.
echo Test the decimator
echo
osgbpp pliers-big.osg -o out.dae --display --convexTM --overall --decPercent .2
echo
echo Decimation tests.
echo Test the Vertex Aggregator.
echo
osgbpp pliers-big.osg -o out.dae --display --convexTM --overall --aggMaxVerts 50


echo
echo
echo
echo Testing the tetra.
echo All should behave normally.
echo
osgbpp tetra.osg -o out.dae --display --convexHull
osgbpp tetra.osg -o out.dae --display --convexTM

echo
echo
echo
echo Testing the concave object.
echo All should behave normally.
echo
osgbpp concave.osg -o out.dae --display --convexHull
osgbpp concave.osg -o out.dae --display --convexHull --overall
osgbpp concave.osg -o out.dae --display --triMesh
osgbpp concave.osg -o out.dae --display --box
osgbpp concave.osg -o out.dae --display --box --overall

echo
echo
echo
echo Testing the cow.
echo All should behave normally.
echo
osgbpp cow.osg -o out.dae --display --convexHull --overall
osgbpp cow.osg -o out.dae --display --triMesh --overall

echo
echo
echo
echo Center of mass tests.
echo First test \(cube\) should behave normally.
echo Next four cubes should behave as if COM is outside lower left corner.
echo
osgbpp offcube.osg -o out.dae --display --box
osgbpp offcube.osg -o out.dae --display --box --com 0,0,0
osgbpp offcube.osg -o out.dae --display --triMesh --com 0,0,0
osgbpp offcube.osg -o out.dae --display --convexTM --com 0,0,0
osgbpp offcube.osg -o out.dae --display --convexHull --com 0,0,0
echo
echo Center of mass tests.
echo Concave object should behave as if COM is outside to the right.
echo Cow should behave as if COM is outside to the right-back.
echo
osgbpp concave.osg -o out.dae --display --box --com 5,0,0
osgbpp cow.osg -o out.dae --display --convexHull --overall --com 2,0,0


echo
echo Decimation combined with center of mass.
echo
osgbpp cow.osg -o out.dae --display --convexTM --overall --simplify .2 --com 4,-.5,0
osgbpp cow.osg -o out.dae --display --convexTM --overall --decPercent .2  --com 4,-.5,0
osgbpp cow.osg -o out.dae --display --convexTM --overall --aggMaxVerts 50  --com 4,-.5,0


echo
echo
echo
echo Test reading collada files
echo Valid keys are 0 and 1.
echo
colladaread tetra.osg
