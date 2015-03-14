

# Using osgbpp #

**osgbpp** creates rigid bodies from OSG data and drops them on a platform. Use this application to test rigid body creation with a variety of control parameters.

## Examples ##
Use **osgbpp** to create a rigid body from dice.osg, one of the **osgBullet** data files:

```
> osgbpp dice.osg
```

To see the model fall again, hit the Delete key. To drag the model, hold down the control key and use the left mouse button.

By default, **osgbpp** creates a Bullet box collision shape. You can visualize the collision shape with the --debug command line option:

```
> osgbpp dice.osg --debug
```

**osgbpp** supports command line patameters to control the collision shape type. For example:

```
> osgbpp dice.osg --debug --cylinder
> osgbpp dice.osg --debug --convexHull
```