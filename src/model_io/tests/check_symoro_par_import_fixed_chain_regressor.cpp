/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

/* Author: Silvio Traversaro */

#include <iDynTree/ModelIO/symoro_par_import.hpp>

#include <kdl_codyco/treeinertialparameters.hpp>
#include <kdl_codyco/treeidsolver_recursive_newton_euler.hpp>
#include <kdl_codyco/treefksolverpos_iterative.hpp>

#include <kdl/treefksolverpos_recursive.hpp>

#include <ctime>

#include <kdl/frames_io.hpp>

using namespace KDL;
using namespace std;
using namespace iDynTree;
using namespace KDL::CoDyCo;

void printLink(const SegmentMap::const_iterator& link, const std::string& prefix)
{
  cout << prefix << "- Segment " << GetTreeElementSegment(link->second).getName() << " has " << GetTreeElementChildren(link->second).size() << " children" << endl;
  cout << " frame_to_tip: " << GetTreeElementSegment(link->second).getFrameToTip() << endl;
  for (unsigned int i=0; i<GetTreeElementChildren(link->second).size(); i++)
    printLink(GetTreeElementChildren(link->second)[i], prefix + "  ");
}



double random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

#include "symoro_generated_fake_puma_regressor.cpp"

int main(int argc, char** argv)
{
  srand(time(NULL));
  if (argc < 2){
    std::cerr << "Expect .par file to parse" << std::endl;
    return -1;
  }

  symoro_par_model mdl;

  if( !parModelFromFile(argv[1],mdl) ) {cerr << "Could not parse SyMoRo par robot model" << endl; return EXIT_FAILURE;}

  std::cout << "Extracted par file" << std::endl;
  std::cout << mdl.toString() << std::endl;


  Tree my_tree;
  if (!treeFromSymoroParFile(argv[1],my_tree,true))
  {cerr << "Could not generate robot model and extract kdl tree" << endl; return EXIT_FAILURE;}

  // walk through tree
  cout << " ======================================" << endl;
  cout << " Tree has " << my_tree.getNrOfSegments() << " link(s) and a root link" << endl;
  cout << " ======================================" << endl;
  SegmentMap::const_iterator root = my_tree.getRootSegment();
  printLink(root, "");

  //Solving regressor in both mode

  JntArray q,dq,ddq,torques,torques_converted;
  std::vector<Wrench> f,f_ext;
  Wrench base_force, base_force_converted;
  Twist base_vel, base_acc;

  q = dq = ddq = torques = torques_converted = JntArray(my_tree.getNrOfJoints());
  f = f_ext = std::vector<Wrench>(my_tree.getNrOfSegments(),KDL::Wrench::Zero());

  double g = 9.8;
  double q_range = 3;
  double dq_range = 1;
  double ddq_range = 1;

  for(size_t i=0; i < my_tree.getNrOfJoints(); i++ )
  {
        q(i) = q_range*random_double();
        dq(i) = dq_range*1*random_double();
        ddq(i) = ddq_range*1*random_double();
  }

  base_vel = Twist::Zero();
  base_acc = Twist(Vector(0,0,-g),Vector::Zero());



  //First we check the geometry
  KDL::Frame H_kdl, H_kdl_rec, H_symoro;
  KDL::CoDyCo::TreeFkSolverPos_iterative pos_slv(my_tree);
  if( pos_slv.JntToCart(KDL::CoDyCo::GeneralizedJntPositions(KDL::Frame(),q),H_kdl,"Link6") != 0 ) { std::cout << "Failed geom solver " << std::endl; return EXIT_FAILURE; }
  symoro_generated_ee_transform(q,H_symoro);


  std::cout << "H_kdl    " << H_kdl << std::endl;
  std::cout << "H_symoro " << H_symoro << std::endl;
  std::cout << (H_kdl.p-H_symoro.p).Norm() << std::endl;

    double tol = 1e-3;


  if( (H_kdl.p-H_symoro.p).Norm() > tol ) { std::cout << "Failed geometry check " << std::endl; }




  Eigen::MatrixXd regr_dirl, regr_symoro;
  regr_dirl.resize(my_tree.getNrOfJoints(),my_tree.getNrOfSegments()*10);
  regr_symoro.resize(my_tree.getNrOfJoints(),my_tree.getNrOfSegments()*10);

  regr_dirl.setZero();

  Eigen::MatrixXd fb_regr_dirl;
  fb_regr_dirl.resize(my_tree.getNrOfJoints()+6,my_tree.getNrOfSegments()*10);

  fb_regr_dirl.setZero();

  //Calculate regressor in both the ways
  TreeInertialParametersRegressor slv(my_tree,Vector(0,0,g));
  //TreeIdSolver_RNE slv_rne(my_tree,Vector(0,0,g));
  //JntArray tau;
  //tau.resize(my_tree.getNrOfJoints());
  regr_symoro.setZero();
  regr_dirl.setZero();

  //if( slv.dynamicsRegressor(q,dq,ddq,regr_dirl) != 0 ) { cout << "dirl regressor failed " << endl; return EXIT_FAILURE; }
  if( slv.dynamicsRegressor(q,dq,ddq,Twist::Zero(),Twist(Vector(0,0,-g),Vector(0,0,0)),fb_regr_dirl) != 0 ) { cout << "fb dirl regressor failed " << endl; return EXIT_FAILURE; }
  if( symoro_generated_fake_puma_regressor(q,dq,ddq,g,regr_symoro) != 0 ) { cout << "SyMoRo regressor failed " << endl; return EXIT_FAILURE; }

  regr_dirl = fb_regr_dirl.block(6,0,regr_dirl.rows(),regr_dirl.cols());
  for(int i=0; i < regr_dirl.rows(); i++ ) {
      for(int j=0; j < regr_dirl.cols(); j++ ) {
          double err;
          err = fabs(regr_dirl(i,j)-regr_symoro(i,j))/fabs(fabs(g)+q_range+dq_range+ddq_range);
          std::cout << "Relative error element " << i << " " << j << " is " << err << " abs(" << regr_dirl(i,j) << " - " << regr_symoro(i,j) << " )"<< std::endl;
          if( err > tol ) return EXIT_FAILURE;
      }
  }

  return EXIT_SUCCESS;
}


