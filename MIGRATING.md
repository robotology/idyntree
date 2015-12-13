# Note on migration 
New versions of iDynTree can change the interface and/or the behaviour 
of existing classes or functions, for several reasons. 

## iDynTree 0.3.X to 0.4.X

1. The `IVector`, `IRawVector`, `IMatrix` and `IRawMatrix` interfaces have been removed for performance reasons, 
   see https://github.com/robotology/idyntree/issues/98#issuecomment-158823148 . If you want to write generic 
   code in C++ you can rely on templates, and on Matlab and Python you can rely on the native dynamic type system. 
   
1. All the core classes that have a fixed size (`Position`, `Rotation`, `Transform`, `SpatialMotionVector`, etc, etc) are 
   not initialized by their empty constructor for performance reasons, see https://github.com/robotology/idyntree/issues/98#issuecomment-158795881 .
   From now on, make sure that initialize them before any use. Most of those classes should have a `zero()` method to
   zero its content, or a `Zero()` static method that return a instance with its content set to zero. 
   For ensuring that no regression happened on the iDynTree codebase, a CMake advanced option `IDYNTREE_RUN_VALGRIND_TESTS`
   has been added to make that no use of initialized memory occurs. 
