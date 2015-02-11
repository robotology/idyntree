/**
* @ingroup idyntree_tutorials
*
* \defgroup icub_genericChainController Controller for a
* Generic Kinematic Chain
*
* A tutorial on how to obtain the parametric model for
*   iCub arm FT sensor measurement.
*
* \author Silvio Traversaro
*
* CopyPolicy: Released under the terms of GPL 2.0 or later
*/

// YARP headers
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>

// iDynTree headers
#include <kdl_codyco/regressors/dynamicRegressorGenerator.hpp>

#include <string>

/*****************************************************************/
int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("name","icubArmRegressor");
    rf.setDefault("config","config.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo() << "Options:\n\n";
        yInfo() << "\t--urdf    file: name of the URDF file containing"
                   " the model of the iCub (default: model.urdf).  \n";

        return 0;
    }

    std::string urdf_filename = rf.findFile("urdf");

    yInfo() << "icubArmRegressor: Tryng to open " << urdf_filename
                                                  << " as iCub model";

    /**
      * The dynamics regressor generator is a class for calculating arbitrary regressor
      * related to robot dynamics, for identification of dynamics parameters, such
      * as inertial parameters (masses, centers of mass, inertia tensor elements) or
      * other related parameters (for example force/torque sensor offset).
      */
    //KDL::CoDyCo::Regressors::DynamicRegressorGenerator ft_regressor_generator;

    return 0;

}

