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

#include <iostream>
#include <fstream>

#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Model/DenavitHartenberg.h>
#include <iDynTree/Model/ModelTestUtils.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/iKinConversions.h>
#include <iCub/iKin/iKinFwd.h>

#include <yarp/os/Property.h>

void printHelp()
{
    std::cerr << "URDF to Denavit-Hartenberg parameter converter." << std::endl;
    std::cerr << "This utility will extract any chain, and it will output" << std::endl;
    std::cerr << "the Denavit-Hartenberg parameters in iKin .ini format." << std::endl;
    std::cerr << "For more information about the iKin .ini format, check:" << std::endl;
    std::cerr << "      http://wiki.icub.org/iCub/main/dox/html/classiCub_1_1iKin_1_1iKinLimb.html#a76c93aae76bb0f7ef9470b81d0da0e26" << std::endl;
    std::cerr << "Usage: urdf2dh robot.urdf base_link_name end_effector_link_name dhParams.ini" << std::endl;
}

bool ExtractReducedJointPosFromFullModel(const iDynTree::Model& fullModel,
                                         const iDynTree::VectorDynSize& fullJntPos,
                                         const iDynTree::Model& reducedModel,
                                               iDynTree::VectorDynSize& reducedJntPos)
{
    assert(fullJntPos.size() == fullModel.getNrOfPosCoords());
    reducedJntPos.resize(reducedModel.getNrOfJoints());

    for (iDynTree::JointIndex reducedJntIdx=0; reducedJntIdx < reducedModel.getNrOfJoints(); reducedJntIdx++)
    {
        // Check that joint is present in the full model
        iDynTree::FrameIndex fullJntIdx = fullModel.getJointIndex(reducedModel.getJointName(reducedJntIdx));
        assert(fullJntIdx != iDynTree::FRAME_INVALID_INDEX);

        // Compute the dofs of the joints and full/reduced model offsets
        assert(fullModel.getJoint(fullJntIdx)->getNrOfDOFs() ==
               reducedModel.getJoint(reducedJntIdx)->getNrOfDOFs());
        unsigned int nrOfDofs =fullModel.getJoint(fullJntIdx)->getNrOfDOFs();
        size_t fullDofOffset = fullModel.getJoint(fullJntIdx)->getDOFsOffset();
        size_t reducedDofOffset = reducedModel.getJoint(reducedJntIdx)->getDOFsOffset();

        // Copy the jnt pos
        for (int j=0; j < nrOfDofs; j++)
        {
            reducedJntPos(reducedDofOffset+j) = fullJntPos(fullDofOffset+j);
        }

    }

    return true;
}

template<typename VectorType1, typename VectorType2>
bool checkVectorsAreEqual(const VectorType1& vec1, const VectorType2& vec2, double tol)
{
    for (unsigned int i = 0; i < vec1.size(); i++ )
    {
        if ( fabs(vec1(i)-vec2(i)) >= tol )
        {
            return false;
        }
    }

    return true;
}

template<typename MatrixType1, typename MatrixType2 >
bool checkMatricesAreEqual(const MatrixType1& mat1, const MatrixType2& mat2, double tol)
{
    for (unsigned int i = 0; i < mat1.rows(); i++)
    {
        for (unsigned int j = 0; j < mat1.rows(); j++)
        {
            if (fabs(mat1(i, j)-mat2(i, j)) >= tol )
            {
                return false;
            }
        }
    }

    return true;
}

bool checkTransformsAreEqual(const iDynTree::Transform& trans1,
                             const iDynTree::Transform& trans2,
                             double tol)
{
    return checkVectorsAreEqual(trans1.getPosition(), trans2.getPosition(), tol) &&
           checkMatricesAreEqual(trans1.getRotation(), trans2.getRotation(), tol);
}

bool checkiKinChainConversion(const iDynTree::Model& model,
                              const std::string& baseFrame, const std::string& distalFrame)
{
    iDynTree::KinDynComputations kinDynComp;

    bool ok = kinDynComp.loadRobotModel(model);

    if (!ok)
    {
        return false;
    }

    // Create a model from a DH chain
    iDynTree::DHChain idynDhChain;
    iDynTree::Model modelExtractedFromChain;
    ok = iDynTree::ExtractDHChainFromModel(model, baseFrame, distalFrame, idynDhChain);
    ok = ok && iDynTree::CreateModelFromDHChain(idynDhChain,
                                                modelExtractedFromChain);

    iDynTree::VectorDynSize fullJntPos, reducedJntPos;

    iDynTree::KinDynComputations chainKinDynComp;
    ok = ok && chainKinDynComp.loadRobotModel(modelExtractedFromChain);

    if (!ok)
    {
        return false;
    }

    for (int i=0; i < 10; i++)
    {
        // Set a random joint state for the model, that will be used
        // to check the transform provided by the DH chain
        iDynTree::getRandomJointPositions(fullJntPos, model);
        kinDynComp.setJointPos(fullJntPos);

        // Extract the state of the Chain from the complete state
        ExtractReducedJointPosFromFullModel(model, fullJntPos, modelExtractedFromChain, reducedJntPos);

        chainKinDynComp.setJointPos(reducedJntPos);

        // Compare the two frames
        ok = ok && checkTransformsAreEqual(kinDynComp.getRelativeTransform(baseFrame, distalFrame),
                                       chainKinDynComp.getRelativeTransform("baseFrame", "distalFrame"), 1e-5);
    }

    return ok;
}

std::string inline int2string(const int inInt)
{
    std::stringstream ss;
    ss.imbue(std::locale::classic());
    ss << inInt;
    return ss.str();
}

int main(int argc, char** argv)
{
  if ( argc == 2 )
  {
      if( (std::string(argv[1]) == "--help" ||
           std::string(argv[1]) == "-h" ) )
      {
          printHelp();
          return 0;
      }
  }
  if (argc != 5)
  {
      printHelp();
      return -1;
  }

  std::string urdf_file_name         = argv[1];
  std::string base_link_name         = argv[2];
  std::string end_effector_link_name = argv[3];
  std::string ikin_ini_file_name     = argv[4];

  //
  // URDF --> iDynTree::Model
  //
  iDynTree::ModelLoader mdlLoader;
  bool ok = mdlLoader.loadModelFromFile(urdf_file_name);
  iDynTree::Model model = mdlLoader.model();

  if (!ok)
  {
      std::cerr << "urdf2dh: error in loading the model" << std::endl;
  }

  //
  // Extract the chain from the model to a iKin limb (i.e. DH parameters)
  //
  iCub::iKin::iKinLimb limb;
  ok = iDynTree::iKinLimbFromModel(model, base_link_name, end_effector_link_name, limb);

  if (!ok)
  {
      std::cerr << "urdf2dh: Could not extract iKinChain" << std::endl;
      return EXIT_FAILURE;
  }


  if (!checkiKinChainConversion(mdlLoader.model(), base_link_name, end_effector_link_name) )
  {
      std::cerr << "urdf2dh error: iDynTree::Model and iKinChain results does not match" << std::endl;
      std::cerr << "urdf2dh: Please open an issue at https://github.com/robotology/idyntree/issues " << std::endl;
      return EXIT_FAILURE;
  }

  yarp::os::Property prop;
  bool result = limb.toLinksProperties(prop);
  if (!result)
  {
      std::cerr << "urdf2dh: Could not export Link Properties from ikin_limb" << std::endl;
      return EXIT_FAILURE;
  } else {
      std::cout << "urdf2dh: Conversion to iKin DH chain completed correctly" << std::endl;
  }

  yarp::os::Bottle prop_bot;
  prop_bot.fromString(prop.toString());

  //Write the properties to file
  std::ofstream ofs (ikin_ini_file_name.c_str(), std::ofstream::out);

  ofs << prop_bot.findGroup("type").toString() << std::endl;
  ofs << prop_bot.findGroup("numLinks").toString() << std::endl;
  ofs << prop_bot.findGroup("H0").toString() << std::endl;
  for (int link = 0; link < limb.getN(); link++)
  {
       std::string link_name = "link_" + int2string(link);
       ofs << prop_bot.findGroup(link_name).toString() << std::endl;

  }
  ofs << prop_bot.findGroup("HN").toString() << std::endl;
  ofs.close();

  return EXIT_SUCCESS;
}


