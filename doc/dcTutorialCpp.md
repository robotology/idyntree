
# DynamicsComputations tutorial

To use the DynamicsComputations class, you have to include its header.
For this tutorial we also use the appropriate `using` definitions to 
avoid having to type the namespace of the class. 

~~~cpp
#include <iostream>

#include <iDynTree/HighLevel/DynamicsComputations.h>

using namespace iDynTree;
using namespace iDynTree::HighLevel;
~~~

The first thing that we have to do 
is to initialize the class by providing a model of the robot. Currently 
iDynTree supports only URDF models to specify the model. 
~~~cpp
int main()
{
    DynamicsComputations dynComp;
    dynComp.loadRobotModelFromFile("model.urdf");
    std::cout << "The loaded model has " << dynComp.getNrOfDegreesOfFreedom() 
          << "internal degrees of freedom and " << dynComp.getNrOfLinks() 
          << "links." << std::endl;
~~~

After that, we have to set the state for the model. There are two different 
methods `setState`, one for the fixed base case and the other for the floating base case. 
In this example we will use the one for the fixed base case, in which you have only 
to specify the joint position, velocity and accelerations and the gravity in the 
base frame. 

~~~cpp
    unsigned int dofs = dynComp.getNrOfDegreesOfFreedom();
    VectorDynSize q(dofs), dq(dofs), ddq(dofs);
    for(unsigned int dof = 0; dof < dofs; dof++ )
    {
        // For the sake of the example, we fill the joints
        // vector with gibberish data (remember in any case
        // that all quantities are expressed in radians-based 
        // units 
        q(dof) = 1.0;
        dq(dof) = 0.4;
        ddq(dof) = 0.3;
    }
    
    // The spatial acceleration is a 6d acceleration vector. 
    // For all 6d quantities, we use the linear-angular serialization
    // (the first three value are for the linear quantity, the 
    //  the last  three values are for the angular quantity)
    SpatialAcc gravity;
    gravity(3) = -9.81;
    dynComp.setRobotState(q,dq,ddq,gravity);
~~~

Once you set the state, you can access the computed dynamical quantities. 
For example you can get the jacobian for a given frame with name "frameName".
Note that "frameName" should be the name of a link in the URDF model. 
Note that the jacobian is the floating base jacobian as described in 
http://wiki.icub.org/codyco/dox/html/dynamics_notation.html . 

~~~cpp
    MatrixDynSize jac(6,6+dofs);
    bool ok = dynComp.getFrameJacobian("frameName",jac);
    
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
Y pi = M(q) d(v)/dt + C(q,v)v + g(q) 
with M(q), C(q,v) and g(q) defined in http://wiki.icub.org/codyco/dox/html/dynamics_notation.html .
The pi vector is a 10 \* nrOfLinks inertial parameters vector, such that the elements of the vector
from the ((i-1) \* 10)-th to the (i \* 10-1)-th belong to the i-th link. For more details on the inertial
parameters vector, please check https://hal.archives-ouvertes.fr/hal-01137215/document . 

~~~cpp
    unsigned int links = dynComp.getNrOfLinks();
    MatrixDynSize regr(6+dofs,10*links);
    ok = dynComp.getDynamicsRegressor(regr);
    
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


