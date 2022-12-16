# Lior and Nave's 3D Animations Assignment 2 (Collision Detection)

***********************************
The assignment is coded in the folder [Assignment2](https://github.com/ThatGuyVanquish/3DAnimation_AS2/tree/master/tutorial/Assignment2)
#    

Our implementation added a CPP file:
[CollisionDetection.cpp](https://github.com/ThatGuyVanquish/3DAnimation_AS2/blob/2567bfb5f1a42b87e493fbdd0228d8def47cc471/tutorial/Assignment2/CollisionDetection.cpp)

Which holds static functions used to calculate the position of the oriented bounding boxes in order to check for collision between two meshes.

### 

The function [*intersects*](https://github.com/ThatGuyVanquish/3DAnimation_AS2/blob/2567bfb5f1a42b87e493fbdd0228d8def47cc471/tutorial/Assignment2/CollisionDetection.cpp#L85) is using the equations from the article to calculate the intersection of boxes in terms of the aligned axes and in term determine whether the two meshes collided.

Using the original axis aligned bounding boxes and the transformation of the mesh object, we use the function [*calcBoxInSpace*](https://github.com/ThatGuyVanquish/3DAnimation_AS2/blob/2567bfb5f1a42b87e493fbdd0228d8def47cc471/tutorial/Assignment2/CollisionDetection.cpp#L64) to calculate the center of the OBB, the set of orthonormal axes and the set of extents used to calculate the radii of the OBB.

## OPTIMIZATIONS
We added two simple optimizations:
1) We used the mesh simplification method from assignment 1 to simplify the mesh and calculate the collisions based on the decimated meshes.
2) The collision check occurs only if the designated 'moving' object has moved. 

1) Changing the value [decimations](https://github.com/ThatGuyVanquish/3DAnimation_AS2/blob/5629ec42e52c8a7878fe9574439d8b32084deb7d/tutorial/Assignment2/BasicScene.cpp#L41) changes the mesh used to calculate collisisons.

2) We keep the previous transformation matrix [prevTransform](https://github.com/ThatGuyVanquish/3DAnimation_AS2/blob/5629ec42e52c8a7878fe9574439d8b32084deb7d/tutorial/Assignment2/BasicScene.h#L33) and recalculate the OBB only if it has [moved](https://github.com/ThatGuyVanquish/3DAnimation_AS2/blob/5629ec42e52c8a7878fe9574439d8b32084deb7d/tutorial/Assignment2/BasicScene.cpp#L127).
