#include <iostream>
#include <cstdio>

#include <kdl_codyco/treeidsolver_recursive_newton_euler.hpp>
#include <iDynTree/ModelIO/impl/urdf_import.hpp>


double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

/**
 * In this example we will perform a classical inverse dynamics on tree structured robot.
 * We will load the kinematic and dynamic parameters of the robot from an URDF file.
 */
int main(int argc, char** argv)
{
    if(argc != 2)
    {
        std::cerr << "tree_inverse_dynamics example usage: ./tree_inverse_dynamics robot.urdf" << std::endl;
        return EXIT_FAILURE;
    }

    std::string urdf_file_name = argv[1];

    //We are using the kdl_format_io library for loading the URDF file to a KDL::Tree object, but if
    //you use ROS you can  use the kdl_parser from the robot_model package (but please note that
    //there are some open issues  with the robot_model kdl_parser : https://github.com/ros/robot_model/pull/66 )
    KDL::Tree my_tree;
    bool urdf_loaded_correctly = iDynTree::treeFromUrdfFile(urdf_file_name.c_str(),my_tree);

    if( !urdf_loaded_correctly )
    {
        std::cerr << "Could not generate robot model and extract kdl tree" << std::endl;
        return EXIT_FAILURE;
    }

    //We will now create a tree inverse dynamics solver
    //This solver performs the classical inverse dynamics
    //so there is a link called floating base that is the one for
    //which the velocity and the acceleration is specified, and
    //where unknown external wrench is assumed applied.
    //The floating base is by default the base link of the URDF, but
    //can be changed with the changeBase method

    KDL::CoDyCo::TreeIdSolver_RNE rne_solver(my_tree);

    //The input variables are :
    // - Joint positions, velocities, accelerations.
    int n_dof = rne_solver.getUndirectedTree().getNrOfDOFs();
    KDL::JntArray q_j(n_dof), dq_j(n_dof), ddq_j(n_dof);

    // - Floating base velocity and (proper) acceleration.
    //   The proper acceleration is the sum of the classical and gravitational acceleration,
    //   i.e. the acceleration that you get reading the output of linear accelerometer.
    KDL::Twist v_base, a_base;

    // - External wrenches applied to the link. This wrenches are expressed in the local reference frame of the link.
    int n_links =  rne_solver.getUndirectedTree().getNrOfLinks();
    std::vector<KDL::Wrench> f_ext(n_links,KDL::Wrench::Zero());

    //The output variables are :
    // - Joint torques.
    KDL::JntArray torques(n_dof);

    // - The base residual wrench (mismatch between external wrenches in input and the model).
    KDL::Wrench f_base;

    //We populate the input variables with random data
    srand(0);
    for(int dof=0; dof < n_dof; dof++)
    {
        q_j(dof) = fRand(-10,10);
        dq_j(dof) = fRand(-10,10);
        ddq_j(dof) = fRand(-10,10);
    }

    //We fill also the linear part of the base velocity
    //but please note that the dynamics is indipendent from it (Galilean relativity)
    for(int i=0; i < 6; i++ )
    {
        v_base[i] = fRand(-10,10);
        a_base[i] = fRand(-10,10);
    }

    //For setting the input wrenches, we first get the index for the link for which we want to set the external wrench.
    //In this example we assume that we have measures for the input wrenches at the four end effectors (two hands, two legs)
    //We use the names defined in REP 120 ( http://www.ros.org/reps/rep-0120.html ) for end effector frames
    //but please change the names if you want to use a different set of links
    int r_gripper_id =  rne_solver.getUndirectedTree().getLink("r_gripper")->getLinkIndex();
    int l_gripper_id =  rne_solver.getUndirectedTree().getLink("l_gripper")->getLinkIndex();
    int r_sole_id =  rne_solver.getUndirectedTree().getLink("r_sole")->getLinkIndex();
    int l_sole_id =  rne_solver.getUndirectedTree().getLink("l_sole")->getLinkIndex();

    for(int i=0; i < 6; i++ )
    {
        f_ext[r_gripper_id][i] = fRand(-10,10);
        f_ext[l_gripper_id][i] = fRand(-10,10);
        f_ext[r_sole_id][i] = fRand(-10,10);
        f_ext[l_sole_id][i] = fRand(-10,10);
    }

    //Now that we have the input, we can compute the inverse dynamics
    int inverse_dynamics_status = rne_solver.CartToJnt(q_j,dq_j,ddq_j,v_base,a_base,f_ext,torques,f_base);

    if( inverse_dynamics_status != 0 )
    {
        std::cerr << "There was an error in the inverse dynamics computations" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Torque computed successfully. " << std::endl;
    std::cout << "Computed torques : " << std::endl;
    for(int dof=0; dof < n_dof; dof++ )
    {
        std::cout << torques(dof) << " ";
    }
    std::cout << std::endl;

    return EXIT_SUCCESS;
}
