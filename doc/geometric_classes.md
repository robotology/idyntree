This tutorial is meant to provide a brief introduction to the basic geometric classes available in iDynTree.
Each section contains the exact same tutorial in a different language. 

## Transform a point
In this tutorial we define some point coordianates (using the `iDynTree::Position` class) 
and a transformation between two frames (using the `iDynTree::Transform` class). We then see how
to transform the point coordinate between a frame and the other.


### Lua 
~~~
require('iDynTree')

-- this is the origin of a the frame B 
-- with respect to the origin of frame A,
-- expresse with the orientation of frame A
origin_A_B = iDynTree.Position(1,2,3) 

-- this is the orientation of frame B
-- with respect to the orientation of frame A,
-- expressed as the rotation matrix that transform
--  a vector expressed in B and returns a vector expressed 
-- in A 
rot_A_B = iDynTree.Rotation( 0,1,0,
                            -1,0,0,
                             0,0,1)

-- we combine this two information in a 
-- transform object, that represent a frame transformation
-- between frame A and frame B
transform_A_B = iDynTree.Transform(rot_A_B,
                                   origin_A_B);
                                   
-- Let's define a point expressed in frame B:
O_B = iDynTree.Position(1,0,0);

-- the same point expressed in A can be obtained as
O_A = transform_A_B*O_B; 

-- However, given that no semantics information was 
-- added to the classes, we can still do stupid things,
-- as using the wrong transform for doing the between the two frames
transform_B_A = transform_A_B:inverse()
O_A_wrong = transform_B_A*O_B; 
~~~
### Matlab

## Transform a point, with semantics checking 

### Lua 
~~~
require('iDynTree')

-- for now everything is encoded with an integer id
A = 1
B = 2
O = 3

-- this is the origin of a the frame B 
-- with respect to the origin of frame A,
-- expresse with the orientation of frame A
origin_A_B = iDynTree.Position(1,2,3) 
-- write semantics 
origin_A_B:getSemantics():setReferencePoint(A)
origin_A_B:getSemantics():setCoordinateFrame(A)
origin_A_B:getSemantics():setPoint(B)

-- this is the orientation of frame B
-- with respect to the orientation of frame A,
-- expressed as the rotation matrix that transform
--  a vector expressed in B and returns a vector expressed 
-- in A 
rot_A_B = iDynTree.Rotation( 0,1,0,
                            -1,0,0,
                             0,0,1)
rot_A_B:getSemantics():setReferenceOrientationFrame(A)
rot_A_B:getSemantics():setOrientationFrame(B)

-- we combine this two information in a 
-- transform object, that represent a frame transformation
-- between frame A and frame B
transform_A_B = iDynTree.Transform(rot_A_B,
                                   origin_A_B);
-- notice that we do not have to specify semantics 
-- for transform_A_B, because everything is already in rot_A_B and origin_A_B 
                                   
-- Let's define a point expressed in frame B:
O_B = iDynTree.Position(10,0,0);
O_B:getSemantics():setReferencePoint(B)
O_B:getSemantics():setCoordinateFrame(B)
O_B:getSemantics():setPoint(O)

-- the same point expressed in A can be obtained as
O_A = transform_A_B*O_B; 
return O_A:toString()

-- Now, doing a incorrect semantic operation will result 
-- in a runtime error, thanks to the semantics checking
transform_B_A = transform_A_B:inverse()
O_A_wrong = transform_B_A*O_B; 
~~~
