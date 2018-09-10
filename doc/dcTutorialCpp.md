
# KinDynComputations tutorial

To use the KinDynComputations class, you have to include its header.
For this tutorial we also use the appropriate `using` definitions to 
avoid having to type the `iDynTree` namespace.

~~~cpp
#include <iostream>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

using namespace iDynTree;
~~~

The first thing that we have to do 
is to initialize the class by providing a model of the robot. Currently 
iDynTree supports only URDF models to specify the model. 
All the logic for loading models, including loading reduced models,
is contained in the iDynTree::ModelLoader class.
~~~cpp
int main()
{
    ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(urdf_filename);

    // Check that ok is actually true

    KinDynComputations kinDynComp;
    kinDynComp.loadRobotModel(mdlLoader.model());
    std::cout << "The loaded model has " << kinDynComp.model().getNrOfDOFs()
          << "internal degrees of freedom and " << kinDynComp.model().getNrOfLinks()
          << "links." << std::endl;
~~~

After that, we have to set the state for the model. There are two different 
methods `setState`, one for the fixed base case and the other for the floating base case. 
In this example we will use the one for the fixed base case, in which you have only 
to specify the joint position, velocity and accelerations and the gravity in the 
base frame. 

~~~cpp
    unsigned int dofs = kinDynComp.model();
    VectorDynSize s(dofs), ds(dofs), dds(dofs);
    for(unsigned int dof = 0; dof < dofs; dof++ )
    {
        // For the sake of the example, we fill the joints
        // vector with gibberish data (remember in any case
        // that all quantities are expressed in radians-based 
        // units 
        s(dof) = 1.0;
        ds(dof) = 0.4;
        dds(dof) = 0.3;
    }
    
    // The gravity acceleration is a 3d vector.
    //
    // For all 6d quantities, we use the linear-angular serialization
    // (the first three value are for the linear quantity, the 
    //  the last  three values are for the angular quantity)
    Vector3 gravity;
    gravity.zero();
    gravity(2) = -9.81;
    kinDynComp.setRobotState(s,ds,gravity);
~~~

Once you set the state, you can access the computed dynamical quantities. 
For example you can get the jacobian for a given frame with name "frameName".
Note that "frameName" should be the name of a link in the URDF model. 

~~~cpp
    MatrixDynSize jac(6,6+dofs);
    bool ok = kinDynComp.getFrameFreeFloatingJacobian("frameName", jac);
    
    if( !ok )
    {
        std::cout << "Error in computing jacobian of frame " << "frameName" << std::endl;
    }
    else 
    {
        std::cout << "Jacobian of frameName is " << std::endl;
        std::cout << jac.toString() << std::endl;
    }
~~~  

You can also get the dynamics regressor. 
The dynamics regressor is a (6+dofs) \* (10 \* nrOfLinks) Y matrix such that:
Y pi = M(q) d(v)/dt + C(q,v)v + g(q) .
The pi vector is a 10 \* nrOfLinks inertial parameters vector, such that the elements of the vector
from the ((i-1) \* 10)-th to the (i \* 10-1)-th belong to the i-th link. For more details on the inertial
parameters vector, please check https://hal.archives-ouvertes.fr/hal-01137215/document . 

~~~cpp
    unsigned int links = dynComp.getNrOfLinks();
    MatrixDynSize regr(6+dofs,10*links);
    Vector6 baseAcc;
    baseAcc.zero();
    ok = kinDynComp.inverseDynamicsInertialParametersRegressor(baseAcc, dds, regr);
    
    if( !ok )
    {
        std::cout << "Error in computing the dynamics regressor" <<std::endl;
    }
    else 
    {
        std::cout << "The dynamics regressor is" << std::endl;
        std::cout << regr.toString() << std::endl;
    }
~~~  

We can then terminate the C++ program as usual.

~~~cpp
    return 0;
}
~~~


