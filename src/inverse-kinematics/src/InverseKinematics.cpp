// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/InverseKinematics.h>
#ifdef IDYNTREE_USES_IPOPT
#include "InverseKinematicsData.h"
#include "TransformConstraint.h"

#include <iDynTree/Axis.h>
#include <iDynTree/Direction.h>
#include <iDynTree/Transform.h>
#include <iDynTree/ModelLoader.h>

#include <iDynTree/EigenHelpers.h>

#include <cassert>
#include <iostream>

// TODO: directly access the raw data, thus removing the methods in IKData class
#define IK_PIMPL(x) static_cast<internal::kinematics::InverseKinematicsData*>((x))

/*
 * Private implementation is divided in two classes.
 * - IKData is responsible of handling all the data and functions which are
 *   tied to a particular solver
 * - IKNLP is an IPOPT NLP implementation. It manages only IPOPT related data
 *   and implements IPOPT related functions
 */
#endif

namespace iDynTree {

#ifndef IDYNTREE_USES_IPOPT
    bool missingIpoptErrorReport() {
        reportError("InverseKinematics", "", "IDYNTREE_USES_IPOPT CMake option need to be set to ON to use InverseKinematics");
        return false;
    }
#endif

    InverseKinematics::InverseKinematics()
    : m_pimpl(0)
    {
#ifdef IDYNTREE_USES_IPOPT
        m_pimpl = new internal::kinematics::InverseKinematicsData();
#else
        missingIpoptErrorReport();
#endif
    }

    InverseKinematics::~InverseKinematics()
    {
#ifdef IDYNTREE_USES_IPOPT
        if (m_pimpl) {
            delete IK_PIMPL(m_pimpl);
            m_pimpl = 0;
        }
#else
        missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::loadModelFromFile(const std::string & filename,
                                              const std::vector<std::string> &consideredJoints,
                                              const std::string & filetype)
    {
#ifdef IDYNTREE_USES_IPOPT
        ModelLoader loader;
        if (!loader.loadModelFromFile(filename) || !loader.isValid()) {
            std::cerr << "[ERROR] iDynTree::InverseDynamics : Failed to load model from URDF file " << filename << std::endl;
            return false;
        }

        return setModel(loader.model(), consideredJoints);
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::setModel(const iDynTree::Model &model,
                                     const std::vector<std::string> &consideredJoints)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->setModel(model, consideredJoints);
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::setJointLimits(std::vector<std::pair<double, double> >& jointLimits)
    {
#ifdef IDYNTREE_USES_IPOPT
      assert(m_pimpl);
      return IK_PIMPL(m_pimpl)->setJointLimits(jointLimits);
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::getJointLimits(std::vector<std::pair<double, double> >& jointLimits)
    {
#ifdef IDYNTREE_USES_IPOPT
      assert(m_pimpl);
      return IK_PIMPL(m_pimpl)->getJointLimits(jointLimits);
#else
        return missingIpoptErrorReport();
#endif
    }

    void InverseKinematics::clearProblem()
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        IK_PIMPL(m_pimpl)->clearProblem();
#else
        missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::setFloatingBaseOnFrameNamed(const std::string &floatingBaseFrameName)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->dynamics().setFloatingBase(floatingBaseFrameName);
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::setCurrentRobotConfiguration(const iDynTree::Transform& baseConfiguration, const iDynTree::VectorDynSize& jointConfiguration)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->setRobotConfiguration(baseConfiguration, jointConfiguration);
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::setCurrentRobotConfiguration(iDynTree::MatrixView<const double> baseConfiguration,
                                                         iDynTree::Span<const double> jointConfiguration)
    {
        constexpr int expected_transform_cols = 4;
        constexpr int expected_transform_rows = 4;
        bool ok = (baseConfiguration.rows() == expected_transform_rows)
            && (baseConfiguration.cols() == expected_transform_cols);
        if( !ok )
        {
            reportError("InverseKinematics","getWorldBaseTransform","Wrong size in input world_T_base");
            return false;
        }

        return this->setCurrentRobotConfiguration(Transform(baseConfiguration), jointConfiguration);
    }

    bool InverseKinematics::setJointConfiguration(const std::string& jointName, const double jointConfiguration)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->setJointConfiguration(jointName, jointConfiguration);
#else
        return missingIpoptErrorReport();
#endif
    }

    void InverseKinematics::setRotationParametrization(enum InverseKinematicsRotationParametrization parametrization)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        IK_PIMPL(m_pimpl)->setRotationParametrization(parametrization);
#else
        missingIpoptErrorReport();
#endif
    }

    enum InverseKinematicsRotationParametrization InverseKinematics::rotationParametrization()
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->rotationParametrization();
#else
        missingIpoptErrorReport();
        return InverseKinematicsRotationParametrizationRollPitchYaw;
#endif
    }

    void InverseKinematics::setMaxIterations(const int max_iter)
    {
#ifdef IDYNTREE_USES_IPOPT
        if (max_iter>0)
            IK_PIMPL(m_pimpl)->m_maxIter = max_iter;
        else
            IK_PIMPL(m_pimpl)->m_maxIter = std::numeric_limits<int>::max();
#else
        missingIpoptErrorReport();
#endif
    }

    int InverseKinematics::maxIterations() const
    {
#ifdef IDYNTREE_USES_IPOPT
        return IK_PIMPL(m_pimpl)->m_maxIter;
#else
        return missingIpoptErrorReport();
#endif
    }

    void InverseKinematics::setMaxCPUTime(const double max_cpu_time)
    {
#ifdef IDYNTREE_USES_IPOPT
        IK_PIMPL(m_pimpl)->m_maxCpuTime = max_cpu_time;
#else
        missingIpoptErrorReport();
#endif
    }

    double InverseKinematics::maxCPUTime() const
    {
#ifdef IDYNTREE_USES_IPOPT
        return IK_PIMPL(m_pimpl)->m_maxCpuTime;
#else
        return missingIpoptErrorReport();
#endif
    }

    void InverseKinematics::setCostTolerance(const double tol)
    {
#ifdef IDYNTREE_USES_IPOPT
        IK_PIMPL(m_pimpl)->m_tol = tol;
#else
        missingIpoptErrorReport();
#endif
    }

    double InverseKinematics::costTolerance() const
    {
#ifdef IDYNTREE_USES_IPOPT
        return IK_PIMPL(m_pimpl)->m_tol;
#else
        return missingIpoptErrorReport();
#endif
    }

    void InverseKinematics::setConstraintsTolerance(const double constr_tol)
    {
#ifdef IDYNTREE_USES_IPOPT
        IK_PIMPL(m_pimpl)->m_constrTol = constr_tol;
#else
        missingIpoptErrorReport();
#endif
    }

    double InverseKinematics::constraintsTolerance() const
    {
#ifdef IDYNTREE_USES_IPOPT
        return IK_PIMPL(m_pimpl)->m_constrTol;
#else
        return missingIpoptErrorReport();
#endif
    }

    void InverseKinematics::setVerbosity(const unsigned int verbose)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        IK_PIMPL(m_pimpl)->m_verbosityLevel = verbose;
#else
        missingIpoptErrorReport();
#endif
    }

    std::string InverseKinematics::linearSolverName()
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->m_solverName;
#else
        missingIpoptErrorReport();
        return {};
#endif
    }

    void InverseKinematics::setLinearSolverName(const std::string &solverName)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        IK_PIMPL(m_pimpl)->m_solverName = solverName;
#else
        missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::addFrameConstraint(const std::string& frameName)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        iDynTree::Transform w_X_frame = IK_PIMPL(m_pimpl)->dynamics().getWorldTransform(frameName);
        return addFrameConstraint(frameName, w_X_frame);
#else
        return missingIpoptErrorReport();
#endif
    }


    bool InverseKinematics::addFrameConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::TransformConstraint::fullTransformConstraint(frameName, constraintValue));
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::addFrameConstraint(const std::string &frameName,
                                               iDynTree::MatrixView<const double> constraintValue)
    {
        constexpr int expected_transform_cols = 4;
        constexpr int expected_transform_rows = 4;
        bool ok = (constraintValue.rows() == expected_transform_rows)
            && (constraintValue.cols() == expected_transform_cols);
        if( !ok )
        {
            reportError("InverseKinematics","addFrameConstraint","Wrong size in input constraintValue");
            return false;
        }

        return this->addFrameConstraint(frameName, Transform(constraintValue));
    }

    bool InverseKinematics::addFramePositionConstraint(const std::string& frameName, const iDynTree::Position& constraintValue)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::TransformConstraint::positionConstraint(frameName, constraintValue));
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::addFramePositionConstraint(const std::string &frameName, iDynTree::Span<const double> constraintValue)
    {
        constexpr int expected_pos_size = 3;
        bool ok = (constraintValue.size() == expected_pos_size);
        if( !ok )
        {
            reportError("InverseKinematics","addFramePositionConstraint","Wrong size in input constraintValue");
            return false;
        }

        return this->addFramePositionConstraint(frameName, Position(constraintValue));
    }

    bool InverseKinematics::addFramePositionConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::TransformConstraint::positionConstraint(frameName, constraintValue.getPosition()));
#else
        return missingIpoptErrorReport();
#endif
    }


    bool InverseKinematics::addFramePositionConstraint(const std::string &frameName,
                                                       iDynTree::MatrixView<const double> constraintValue)
    {
        constexpr int expected_transform_cols = 4;
        constexpr int expected_transform_rows = 4;
        bool ok = (constraintValue.rows() == expected_transform_rows)
            && (constraintValue.cols() == expected_transform_cols);
        if( !ok )
        {
            reportError("InverseKinematics","addFrameConstraint","Wrong size in input constraintValue");
            return false;
        }

        return this->addFramePositionConstraint(frameName, Transform(constraintValue));
    }

    bool InverseKinematics::addFrameRotationConstraint(const std::string& frameName, const iDynTree::Rotation& constraintValue)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::TransformConstraint::rotationConstraint(frameName, constraintValue));
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::addFrameRotationConstraint(const std::string& frameName, const iDynTree::Transform& constraintValue)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addFrameConstraint(internal::kinematics::TransformConstraint::rotationConstraint(frameName, constraintValue.getRotation()));
#else
        return missingIpoptErrorReport();
#endif
    }

      bool InverseKinematics::addFrameRotationConstraint(const std::string& frameName,
                                                         iDynTree::MatrixView<const double> constraintValue)
      {
          constexpr int expected_transform_cols = 4;
          constexpr int expected_transform_rows = 4;
          constexpr int expected_rotation_cols = 3;
          constexpr int expected_rotation_rows = 3;

          if ((constraintValue.rows() == expected_transform_rows)
              && (constraintValue.cols() == expected_transform_cols))
          {
              return this->addFrameRotationConstraint(frameName, Transform(constraintValue));
          }

          if ((constraintValue.rows() == expected_rotation_rows)
              && (constraintValue.cols() == expected_rotation_cols))
          {
              return this->addFrameRotationConstraint(frameName, Rotation(constraintValue));
          }

          reportError("InverseKinematics",
                      "addFrameRotationConstraint",
                      "Wrong size in input constraintValue");
          return false;
      }

    bool InverseKinematics::activateFrameConstraint(const std::string& frameName, const Transform& newConstraintValue)
    {
#ifdef IDYNTREE_USES_IPOPT
        iDynTree::LinkIndex frameIndex = IK_PIMPL(m_pimpl)->m_dynamics.getFrameIndex(frameName);
        if (frameIndex < 0)
        {
            return false;
        }

        internal::kinematics::TransformMap::iterator it = IK_PIMPL(m_pimpl)->m_constraints.find(frameIndex);
        if (it == IK_PIMPL(m_pimpl)->m_constraints.end())
        {
            return false;
        }

        it->second.setActive(true);
        it->second.setPosition(newConstraintValue.getPosition());
        it->second.setRotation(newConstraintValue.getRotation());

        // The problem needs to be reinitialized
        IK_PIMPL(m_pimpl)->m_problemInitialized = false;

        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::activateFrameConstraint(const std::string& frameName,
                                                    iDynTree::MatrixView<const double> newConstraintValue)
    {
        constexpr int expected_transform_cols = 4;
        constexpr int expected_transform_rows = 4;
        bool ok = (newConstraintValue.rows() == expected_transform_rows)
            && (newConstraintValue.cols() == expected_transform_cols);
        if( !ok )
        {
            reportError("InverseKinematics","activateFrameConstraint","Wrong size in input constraintValue");
            return false;
        }

        return this->activateFrameConstraint(frameName, Transform(newConstraintValue));
    }

    bool InverseKinematics::deactivateFrameConstraint(const std::string& frameName)
    {
#ifdef IDYNTREE_USES_IPOPT
        iDynTree::LinkIndex frameIndex = IK_PIMPL(m_pimpl)->m_dynamics.getFrameIndex(frameName);
        if (frameIndex < 0)
        {
            return false;
        }

        internal::kinematics::TransformMap::iterator it = IK_PIMPL(m_pimpl)->m_constraints.find(frameIndex);
        if (it == IK_PIMPL(m_pimpl)->m_constraints.end())
        {
            return false;
        }

        it->second.setActive(false);

        // The problem needs to be reinitialized
        IK_PIMPL(m_pimpl)->m_problemInitialized = false;

        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    bool  InverseKinematics::isFrameConstraintActive(const std::string& frameName) const
    {
#ifdef IDYNTREE_USES_IPOPT
        iDynTree::LinkIndex frameIndex = IK_PIMPL(m_pimpl)->m_dynamics.getFrameIndex(frameName);
        if (frameIndex < 0)
        {
            return false;
        }

        internal::kinematics::TransformMap::iterator it = IK_PIMPL(m_pimpl)->m_constraints.find(frameIndex);
        if (it == IK_PIMPL(m_pimpl)->m_constraints.end())
        {
            return false;
        }

        return it->second.isActive();
#else
        return missingIpoptErrorReport();
#endif
    }


    bool InverseKinematics::addCenterOfMassProjectionConstraint(const std::string &firstSupportFrame,
                                                                const Polygon &firstSupportPolygon,
                                                                const iDynTree::Direction xAxisOfPlaneInWorld,
                                                                const iDynTree::Direction yAxisOfPlaneInWorld,
                                                                const iDynTree::Position originOfPlaneInWorld)
    {
#ifdef IDYNTREE_USES_IPOPT
        std::vector<std::string> supportFrames;
        std::vector<Polygon> supportPolygons;
        supportFrames.push_back(firstSupportFrame);
        supportPolygons.push_back(firstSupportPolygon);
        return addCenterOfMassProjectionConstraint(supportFrames,supportPolygons,xAxisOfPlaneInWorld,yAxisOfPlaneInWorld,originOfPlaneInWorld);
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::addCenterOfMassProjectionConstraint(const std::string &firstSupportFrame,
                                                                const Polygon &firstSupportPolygon,
                                                                const std::string &secondSupportFrame,
                                                                const Polygon &secondSupportPolygon,
                                                                const iDynTree::Direction xAxisOfPlaneInWorld,
                                                                const iDynTree::Direction yAxisOfPlaneInWorld,
                                                                const iDynTree::Position originOfPlaneInWorld)
    {
#ifdef IDYNTREE_USES_IPOPT
        std::vector<std::string> supportFrames;
        std::vector<Polygon> supportPolygons;
        supportFrames.push_back(firstSupportFrame);
        supportFrames.push_back(secondSupportFrame);
        supportPolygons.push_back(firstSupportPolygon);
        supportPolygons.push_back(secondSupportPolygon);
        return addCenterOfMassProjectionConstraint(supportFrames,supportPolygons,xAxisOfPlaneInWorld,yAxisOfPlaneInWorld,originOfPlaneInWorld);
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::addCenterOfMassProjectionConstraint(const std::vector<std::string> &supportFrames,
                                                                const std::vector<Polygon> &supportPolygons,
                                                                const iDynTree::Direction xAxisOfPlaneInWorld,
                                                                const iDynTree::Direction yAxisOfPlaneInWorld,
                                                                const iDynTree::Position originOfPlaneInWorld)
    {
#ifdef IDYNTREE_USES_IPOPT
        if( supportFrames.size() == 0 )
        {
            reportError("InverseKinematics","addCenterOfMassProjectionConstraint","No support frames specified");
            return false;
        }

        if( supportFrames.size() != supportPolygons.size() )
        {
            reportError("InverseKinematics","addCenterOfMassProjectionConstraint","Size mismatch between supportFrames and supportPolygons");
            return false;
        }


        size_t nrOfSupportLinks = supportFrames.size();
        IK_PIMPL(m_pimpl)->m_comHullConstraint_supportFramesIndeces.resize(nrOfSupportLinks);

        for (int i=0; i < nrOfSupportLinks; i++)
        {
            // check for transform
            int frameIndex = IK_PIMPL(m_pimpl)->m_dynamics.getFrameIndex(supportFrames[i]);
            if (frameIndex  == iDynTree::FRAME_INVALID_INDEX)
            {
                std::stringstream ss;
                ss << "Frame " << supportFrames[i] << " not found in the model";
                reportError("InverseKinematics","addCenterOfMassProjectionConstraint",ss.str().c_str());
                return false;
            }

            IK_PIMPL(m_pimpl)->m_comHullConstraint_supportFramesIndeces[i] = frameIndex;

            internal::kinematics::TransformMap::iterator constraintIt = IK_PIMPL(m_pimpl)->m_constraints.find(frameIndex);
            if (constraintIt == IK_PIMPL(m_pimpl)->m_constraints.end())
            {
                std::stringstream ss;
                ss << "Frame " << supportFrames[i] << " is not subject to a constraint";
                reportError("InverseKinematics","addCenterOfMassProjectionConstraint",ss.str().c_str());
                return false;
            }
        }

        // Initialize the COM's projection direction in such a way that is along the lien perpendicular to the xy-axes of the World
	    iDynTree::Direction zAxisOfPlaneInWorld;
        toEigen(zAxisOfPlaneInWorld) = toEigen(xAxisOfPlaneInWorld).cross(toEigen(yAxisOfPlaneInWorld));

        IK_PIMPL(m_pimpl)->m_comHullConstraint_projDirection = zAxisOfPlaneInWorld;

        // Configuration went fine, enable constraint and save the parameters
        IK_PIMPL(m_pimpl)->m_comHullConstraint.setActive(true);
        IK_PIMPL(m_pimpl)->m_comHullConstraint_supportPolygons = supportPolygons;
        IK_PIMPL(m_pimpl)->m_comHullConstraint_xAxisOfPlaneInWorld = xAxisOfPlaneInWorld;
        IK_PIMPL(m_pimpl)->m_comHullConstraint_yAxisOfPlaneInWorld = yAxisOfPlaneInWorld;
        IK_PIMPL(m_pimpl)->m_comHullConstraint_originOfPlaneInWorld = originOfPlaneInWorld;

        // If this method is called again to reconfigure the constraint, the problem needs to be reinitialized
        IK_PIMPL(m_pimpl)->m_problemInitialized = false;

        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    double InverseKinematics::getCenterOfMassProjectionMargin()
    {
#ifdef IDYNTREE_USES_IPOPT
        if (!IK_PIMPL(m_pimpl)->m_problemInitialized) {
            IK_PIMPL(m_pimpl)->computeProblemSizeAndResizeBuffers();
        }

        // Compute center of mass in the first constraint frame
        iDynTree::KinDynComputations & kinDyn = IK_PIMPL(m_pimpl)->m_dynamics;
        assert(IK_PIMPL(m_pimpl)->m_comHullConstraint.supportFrameIndices.size() > 0);
        iDynTree::Position comInAbsoluteConstraintFrame =
            IK_PIMPL(m_pimpl)->m_comHullConstraint.absoluteFrame_X_supportFrame[0]*(kinDyn.getWorldTransform(IK_PIMPL(m_pimpl)->m_comHullConstraint.supportFrameIndices[0]).inverse()*kinDyn.getCenterOfMassPosition());

        iDynTree::Vector2 comProjection = IK_PIMPL(m_pimpl)->m_comHullConstraint.projectAlongDirection(comInAbsoluteConstraintFrame);
        return IK_PIMPL(m_pimpl)->m_comHullConstraint.computeMargin(comProjection);
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::getCenterOfMassProjectConstraintConvexHull(Polygon2D& poly)
    {
#ifdef IDYNTREE_USES_IPOPT
        if (!IK_PIMPL(m_pimpl)->m_comHullConstraint.isActive())
        {
            poly.setNrOfVertices(0);
            return false;
        }

        if (!IK_PIMPL(m_pimpl)->m_problemInitialized) {
            IK_PIMPL(m_pimpl)->computeProblemSizeAndResizeBuffers();
        }

        poly = IK_PIMPL(m_pimpl)->m_comHullConstraint.projectedConvexHull;
        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::addTarget(const std::string& frameName,
                                      const iDynTree::Transform& constraintValue,
                                      const double positionWeight,
                                      const double rotationWeight)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::TransformConstraint::fullTransformConstraint(frameName,
                                                                                                               constraintValue,
                                                                                                               positionWeight,
                                                                                                               rotationWeight));
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::addTarget(const std::string& frameName,
                                      iDynTree::MatrixView<const double> targetValue,
                                      const double positionWeight,
                                      const double rotationWeight)
    {
        constexpr int expected_transform_cols = 4;
        constexpr int expected_transform_rows = 4;
        bool ok = (targetValue.rows() == expected_transform_rows)
                  && (targetValue.cols() == expected_transform_cols);
        if (!ok)
        {
            reportError("InverseKinematics",
                        "addTarget",
                        "Wrong size in input targetValue");
            return false;
        }

        return this->addTarget(frameName, Transform(targetValue), positionWeight, rotationWeight);
    }

    bool InverseKinematics::addPositionTarget(const std::string& frameName, const iDynTree::Position& constraintValue, const double positionWeight)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::TransformConstraint::positionConstraint(frameName,  constraintValue, positionWeight));
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::addPositionTarget(const std::string& frameName,
                                              iDynTree::Span<const double> targetValue,
                                              const double positionWeight)
    {
        constexpr int expected_pos_size = 3;
        bool ok = (targetValue.size() == expected_pos_size);
        if (!ok)
        {
            reportError("InverseKinematics",
                        "addPositionTarget",
                        "Wrong size in input targetValue");
            return false;
        }

        return this->addPositionTarget(frameName, Position(targetValue), positionWeight);
    }

    bool InverseKinematics::addPositionTarget(const std::string& frameName, const iDynTree::Transform& constraintValue, const double positionWeight)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::TransformConstraint::positionConstraint(frameName,  constraintValue.getPosition(), positionWeight));
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::addPositionTarget(const std::string& frameName,
                                              iDynTree::MatrixView<const double> targetValue,
                                              const double positionWeight)
    {
        constexpr int expected_transform_cols = 4;
        constexpr int expected_transform_rows = 4;
        bool ok = (targetValue.rows() == expected_transform_rows)
                  && (targetValue.cols() == expected_transform_cols);
        if (!ok)
        {
            reportError("InverseKinematics",
                        "addPositionTarget",
                        "Wrong size in input targetValue");
            return false;
        }

        return this->addPositionTarget(frameName, Transform(targetValue), positionWeight);
    }

    bool InverseKinematics::addRotationTarget(const std::string& frameName, const iDynTree::Rotation& constraintValue, const double rotationWeight)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::TransformConstraint::rotationConstraint(frameName,  constraintValue, rotationWeight));
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::addRotationTarget(const std::string& frameName, const iDynTree::Transform& constraintValue, const double rotationWeight)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->addTarget(internal::kinematics::TransformConstraint::rotationConstraint(frameName,  constraintValue.getRotation(), rotationWeight));
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::addRotationTarget(const std::string& frameName,
                                              iDynTree::MatrixView<const double> targetValue,
                                              const double rotationWeight)
    {
        constexpr int expected_transform_cols = 4;
        constexpr int expected_transform_rows = 4;
        constexpr int expected_rotation_cols = 3;
        constexpr int expected_rotation_rows = 3;

        if ((targetValue.rows() == expected_transform_rows)
            && (targetValue.cols() == expected_transform_cols))
        {
            return this->addRotationTarget(frameName, Transform(targetValue), rotationWeight);
        }

        if ((targetValue.rows() == expected_rotation_rows)
            && (targetValue.cols() == expected_rotation_cols))
        {
            return this->addRotationTarget(frameName,Rotation(targetValue),rotationWeight);
        }

        reportError("InverseKinematics",
                    "addRotationTarget",
                    "Wrong size in input targetValue");
        return false;
    }

    bool InverseKinematics::updateTarget(const std::string& frameName,
                                         const Transform& targetValue,
                                         const double positionWeight,
                                         const double rotationWeight)
    {
#ifdef IDYNTREE_USES_IPOPT
        internal::kinematics::TransformMap::iterator transConstr = IK_PIMPL(m_pimpl)->getTargetRefIfItExists(frameName);

        if( transConstr == IK_PIMPL(m_pimpl)->m_targets.end() )
        {
            std::stringstream ss;
            ss << "No target for frame " << frameName << " was added to the InverseKinematics problem.";
            reportError("InverseKinematics","updateTarget",ss.str().c_str());
            return false;
        }

        IK_PIMPL(m_pimpl)->updatePositionTarget(transConstr,targetValue.getPosition(),positionWeight);
        IK_PIMPL(m_pimpl)->updateRotationTarget(transConstr,targetValue.getRotation(),rotationWeight);
        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::updateTarget(const std::string& frameName,
                                         iDynTree::MatrixView<const double> targetValue,
                                         const double positionWeight,
                                         const double rotationWeight)
    {
        constexpr int expected_transform_cols = 4;
        constexpr int expected_transform_rows = 4;
        bool ok = (targetValue.rows() == expected_transform_rows)
                  && (targetValue.cols() == expected_transform_cols);
        if (!ok)
        {
            reportError("InverseKinematics",
                        "updateTarget",
                        "Wrong size in input targetValue");
            return false;
        }

        return this->updateTarget(frameName, Transform(targetValue), positionWeight, rotationWeight);
    }

    bool InverseKinematics::updatePositionTarget(const std::string& frameName,
                                                 const Position& targetValue,
                                                 const double positionWeight)
    {
#ifdef IDYNTREE_USES_IPOPT
        internal::kinematics::TransformMap::iterator transConstr = IK_PIMPL(m_pimpl)->getTargetRefIfItExists(frameName);

        if( transConstr == IK_PIMPL(m_pimpl)->m_targets.end() )
        {
            std::stringstream ss;
            ss << "No target for frame " << frameName << " was added to the InverseKinematics problem.";
            reportError("InverseKinematics","updatePositionTarget",ss.str().c_str());
            return false;
        }

        IK_PIMPL(m_pimpl)->updatePositionTarget(transConstr,targetValue,positionWeight);
        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::updatePositionTarget(const std::string& frameName,
                                                 iDynTree::Span<const double> targetValue,
                                                 const double positionWeight)
    {
        constexpr int expected_pos_size = 3;
        bool ok = (targetValue.size() == expected_pos_size);
        if (!ok)
        {
            reportError("InverseKinematics",
                        "updatePositionTarget",
                        "Wrong size in input targetValue");
            return false;
        }

        return this->updatePositionTarget(frameName, Position(targetValue), positionWeight);
    }

    bool InverseKinematics::updateRotationTarget(const std::string& frameName,
                                                 const Rotation& targetValue,
                                                 const double rotationWeight)
    {
#ifdef IDYNTREE_USES_IPOPT
        internal::kinematics::TransformMap::iterator transConstr = IK_PIMPL(m_pimpl)->getTargetRefIfItExists(frameName);

        if( transConstr == IK_PIMPL(m_pimpl)->m_targets.end() )
        {
            std::stringstream ss;
            ss << "No target for frame " << frameName << " was added to the InverseKinematics problem.";
            reportError("InverseKinematics","updateRotationTarget",ss.str().c_str());
            return false;
        }

        IK_PIMPL(m_pimpl)->updateRotationTarget(transConstr,targetValue,rotationWeight);
        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::updateRotationTarget(const std::string& frameName,
                                                 iDynTree::MatrixView<const double> targetValue,
                                                 const double rotationWeight)
    {
        constexpr int expected_rotation_cols = 3;
        constexpr int expected_rotation_rows = 3;

        const bool ok = ((targetValue.rows() == expected_rotation_rows)
            && (targetValue.cols() == expected_rotation_cols));

        if (!ok)
        {
            reportError("InverseKinematics",
                        "updateRotationTarget",
                        "Wrong size in input targetValue");
            return false;
        }

        return this->updateRotationTarget(frameName,Rotation(targetValue),rotationWeight);
    }

    bool InverseKinematics::setDesiredFullJointsConfiguration(iDynTree::Span<const double> desiredJointConfiguration,
                                                              double weight)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        assert(IK_PIMPL(m_pimpl)->m_preferredJointsConfiguration.size() == desiredJointConfiguration.size());
        IK_PIMPL(m_pimpl)->m_preferredJointsConfiguration = desiredJointConfiguration;
        if (weight >= 0.0) {
            iDynTree::toEigen(IK_PIMPL(m_pimpl)->m_preferredJointsWeight).setConstant(weight);
        }
        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::setDesiredFullJointsConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration,
                                                              double weight)
    {
        return this->setDesiredFullJointsConfiguration(make_span(desiredJointConfiguration), weight);
    }

    bool InverseKinematics::setDesiredFullJointsConfiguration(iDynTree::Span<const double> desiredJointConfiguration,
                                                              iDynTree::Span<const double> weights)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        assert(IK_PIMPL(m_pimpl)->m_preferredJointsConfiguration.size() == desiredJointConfiguration.size());

        if (desiredJointConfiguration.size() != weights.size()){
            reportError("InverseKinematics", "setDesiredFullJointsConfiguration", "The dimension of the desired weights is different from the desiredJointConfiguration size.");
            return false;
        }

        IK_PIMPL(m_pimpl)->m_preferredJointsConfiguration = desiredJointConfiguration;

        assert(IK_PIMPL(m_pimpl)->m_preferredJointsWeight.size() == weights.size());
        for (unsigned int i = 0; i < weights.size(); ++i){
            if (weights[i] >= 0.0) {
                IK_PIMPL(m_pimpl)->m_preferredJointsWeight(i) = weights[i];
            }
        }
        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::setDesiredFullJointsConfiguration(
        const iDynTree::VectorDynSize& desiredJointConfiguration,
        const iDynTree::VectorDynSize& weights)
    {
        return this->setDesiredFullJointsConfiguration(make_span(desiredJointConfiguration),
                                                       make_span(weights));
    }

    bool InverseKinematics::setDesiredReducedJointConfiguration(iDynTree::Span<const double> desiredJointConfiguration,
                                                                double weight)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        assert(IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints.size() == desiredJointConfiguration.size());
        for (size_t i = 0; i < desiredJointConfiguration.size(); ++i) {
            IK_PIMPL(m_pimpl)->m_preferredJointsConfiguration(IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]) = desiredJointConfiguration[i];
        }

        if (weight >= 0.0) {
            iDynTree::toEigen(IK_PIMPL(m_pimpl)->m_preferredJointsWeight).setConstant(weight);
        }
        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::setDesiredReducedJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration,
                                                                double weight)
    {
        return this->setDesiredReducedJointConfiguration(make_span(desiredJointConfiguration), weight);
    }

    bool InverseKinematics::setDesiredReducedJointConfiguration(iDynTree::Span<const double> desiredJointConfiguration,
                                                                iDynTree::Span<const double> weights)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        assert(IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints.size() == desiredJointConfiguration.size());

        if (desiredJointConfiguration.size() != weights.size()){
            reportError("InverseKinematics", "setDesiredFullJointsConfiguration", "The dimension of the desired weights is different from the desiredJointConfiguration size.");
            return false;
        }

        for (size_t i = 0; i < desiredJointConfiguration.size(); ++i) {
            IK_PIMPL(m_pimpl)->m_preferredJointsConfiguration(IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]) = desiredJointConfiguration[i];
            if (weights(i) >= 0.0) {
                IK_PIMPL(m_pimpl)->m_preferredJointsWeight(IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]) = weights[i];
            }
        }

        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::setDesiredReducedJointConfiguration(const VectorDynSize &desiredJointConfiguration,
                                                                const VectorDynSize &weights)
    {
        return this->setDesiredReducedJointConfiguration(make_span(desiredJointConfiguration),
                                                         make_span(weights));
    }

    bool InverseKinematics::setFullJointsInitialCondition(const iDynTree::Transform* baseTransform,
                                                          const iDynTree::VectorDynSize* initialCondition)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        if (baseTransform) {
            IK_PIMPL(m_pimpl)->m_baseInitialCondition = *baseTransform;
            IK_PIMPL(m_pimpl)->m_areBaseInitialConditionsSet = true;
        }
        if (initialCondition) {
            assert(initialCondition->size() == IK_PIMPL(m_pimpl)->m_jointInitialConditions.size());
            IK_PIMPL(m_pimpl)->m_jointInitialConditions = *initialCondition;
            IK_PIMPL(m_pimpl)->m_areJointsInitialConditionsSet = internal::kinematics::InverseKinematicsData::InverseKinematicsInitialConditionFull;
        }
        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::setReducedInitialCondition(const iDynTree::Transform* baseTransform,
                                                       const iDynTree::VectorDynSize* initialCondition)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        if (baseTransform) {
            IK_PIMPL(m_pimpl)->m_baseInitialCondition = *baseTransform;
            IK_PIMPL(m_pimpl)->m_areBaseInitialConditionsSet = true;
        }
        if (initialCondition) {
            // This function passes a guess for the ik for the optimised joints only
            assert(initialCondition->size() == IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints.size());
            for (size_t i = 0; i < initialCondition->size(); ++i) {
                IK_PIMPL(m_pimpl)->m_jointInitialConditions(IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]) = (*initialCondition)(i);
            }
            IK_PIMPL(m_pimpl)->m_areJointsInitialConditionsSet = internal::kinematics::InverseKinematicsData::InverseKinematicsInitialConditionPartial;
        }
        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    void InverseKinematics::setDefaultTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraint mode)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        IK_PIMPL(m_pimpl)->setDefaultTargetResolutionMode(mode);
#else
        missingIpoptErrorReport();
#endif
    }

    enum iDynTree::InverseKinematicsTreatTargetAsConstraint InverseKinematics::defaultTargetResolutionMode()
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->defaultTargetResolutionMode();
#else
        missingIpoptErrorReport();
        return InverseKinematicsTreatTargetAsConstraintNone;
#endif
    }

    bool InverseKinematics::setTargetResolutionMode(const std::string& frameName, InverseKinematicsTreatTargetAsConstraint mode)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        internal::kinematics::TransformMap::iterator transConstr = IK_PIMPL(m_pimpl)->getTargetRefIfItExists(frameName);

        if( transConstr == IK_PIMPL(m_pimpl)->m_targets.end() )
        {
            std::stringstream ss;
            ss << "No target for frame " << frameName << " was added to the InverseKinematics problem.";
            reportError("InverseKinematics","setTargetResolutionMode",ss.str().c_str());
            return false;
        }

        IK_PIMPL(m_pimpl)->setTargetResolutionMode(transConstr, mode);
        return true;
#else
        return missingIpoptErrorReport();
#endif
    }


    enum InverseKinematicsTreatTargetAsConstraint InverseKinematics::targetResolutionMode(const std::string& frameName)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        internal::kinematics::TransformMap::iterator transConstr = IK_PIMPL(m_pimpl)->getTargetRefIfItExists(frameName);

        if( transConstr == IK_PIMPL(m_pimpl)->m_targets.end() )
        {
            std::stringstream ss;
            ss << "No target for frame " << frameName << " was added to the InverseKinematics problem.";
            reportError("InverseKinematics","targetResolutionMode",ss.str().c_str());
            return InverseKinematicsTreatTargetAsConstraintNone;
        }

        return IK_PIMPL(m_pimpl)->targetResolutionMode(transConstr);
#else
        missingIpoptErrorReport();
        return InverseKinematicsTreatTargetAsConstraintNone;
#endif
    }

    bool InverseKinematics::solve()
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->solveProblem();
#else
        return missingIpoptErrorReport();
#endif
    }

    void InverseKinematics::getFullJointsSolution(iDynTree::Transform & baseTransformSolution,
                                                  iDynTree::VectorDynSize & shapeSolution)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        assert(shapeSolution.size() == IK_PIMPL(m_pimpl)->m_dofs);
        baseTransformSolution = IK_PIMPL(m_pimpl)->m_baseResults;
        shapeSolution         = IK_PIMPL(m_pimpl)->m_jointsResults;
#else
        missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::getFullJointsSolution(iDynTree::MatrixView<double> baseTransformSolution,
                                                  iDynTree::Span<double> shapeSolution)
    {
#ifdef IDYNTREE_USES_IPOPT
        bool ok = shapeSolution.size() == IK_PIMPL(m_pimpl)->m_dofs;
        if (!ok)
        {
            reportError("InveseKineamtics",
                        "getFullJointsSolution",
                        "Invalid size of the shapeSolution vector");
            return false;
        }

        constexpr int expected_transform_cols = 4;
        constexpr int expected_transform_rows = 4;
        ok = (baseTransformSolution.rows() == expected_transform_rows)
            && (baseTransformSolution.cols() == expected_transform_cols);
        if (!ok)
        {
            reportError("InverseKinematics",
                        "getFullJointsSolution",
                        "Invalid size of the baseTransformSolution vector");
            return false;
        }

        toEigen(baseTransformSolution) = toEigen(IK_PIMPL(m_pimpl)->m_baseResults.asHomogeneousTransform());
        toEigen(shapeSolution)         = toEigen(IK_PIMPL(m_pimpl)->m_jointsResults);
        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    void InverseKinematics::getReducedSolution(iDynTree::Transform & baseTransformSolution,
                                               iDynTree::VectorDynSize & shapeSolution)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        baseTransformSolution = IK_PIMPL(m_pimpl)->m_baseResults;
        assert(shapeSolution.size() == IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints.size());
        for (size_t i = 0; i < shapeSolution.size(); ++i) {
            shapeSolution(i) = IK_PIMPL(m_pimpl)->m_jointsResults(IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]);
        }
        return;
#else
        missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::getReducedSolution(iDynTree::MatrixView<double> baseTransformSolution,
                                               iDynTree::Span<double> shapeSolution)
    {
#ifdef IDYNTREE_USES_IPOPT
        bool ok = shapeSolution.size() == IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints.size();
        if (!ok)
        {
            reportError("InveseKineamtics",
                        "getReducedSolution",
                        "Invalid size of the shapeSolution vector");
            return false;
        }

        constexpr int expected_transform_cols = 4;
        constexpr int expected_transform_rows = 4;
        ok = (baseTransformSolution.rows() == expected_transform_rows)
            && (baseTransformSolution.cols() == expected_transform_cols);
        if (!ok)
        {
            reportError("InverseKinematics",
                        "getReducedSolution",
                        "Invalid size of the baseTransformSolution vector");
            return false;
        }

        assert(m_pimpl);
        toEigen(baseTransformSolution) = toEigen(IK_PIMPL(m_pimpl)->m_baseResults.asHomogeneousTransform());
        assert(shapeSolution.size() == IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints.size());
        for (size_t i = 0; i < shapeSolution.size(); ++i) {
            shapeSolution(i) = IK_PIMPL(m_pimpl)->m_jointsResults(IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.modelJointsToOptimisedJoints[i]);
        }
        return true;
#else
        return missingIpoptErrorReport();
#endif
    }


    bool InverseKinematics::getPoseForFrame(const std::string& frameName,
                                            iDynTree::Transform& transform)
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        transform = IK_PIMPL(m_pimpl)->dynamics().getWorldTransform(frameName);
        return true;
#else
        return missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::getPoseForFrame(const std::string& frameName,
                                            iDynTree::MatrixView<double> transform)
    {
        constexpr int expected_transform_cols = 4;
        constexpr int expected_transform_rows = 4;
        const bool ok = (transform.rows() == expected_transform_rows)
            && (transform.cols() == expected_transform_cols);
        if (!ok)
        {
            reportError("InverseKinematics",
                        "getPoseForFrame",
                        "Invalid size of the transform matrix");
            return false;
        }

#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        toEigen(transform) = toEigen(IK_PIMPL(m_pimpl)->dynamics().getWorldTransform(frameName).asHomogeneousTransform());
        return true;
#else
        return missingIpoptErrorReport();
#endif
    }


    const Model& InverseKinematics::fullModel() const
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->m_dynamics.model();
#else
        missingIpoptErrorReport();
        return this->reducedModel();
#endif
    }

    const Model& InverseKinematics::reducedModel() const
    {
#ifdef IDYNTREE_USES_IPOPT
        assert(m_pimpl);
        return IK_PIMPL(m_pimpl)->m_reducedVariablesInfo.reducedModel;
#else
        missingIpoptErrorReport();
        return this->reducedModel();
#endif
    }

    bool InverseKinematics::isCOMTargetActive()
    {
#ifdef IDYNTREE_USES_IPOPT
        return IK_PIMPL(m_pimpl)->isCoMTargetActive();
#else
        return missingIpoptErrorReport();
#endif
    }

    void InverseKinematics::setCOMAsConstraint(bool asConstraint)
    {
#ifdef IDYNTREE_USES_IPOPT
        IK_PIMPL(m_pimpl)->setCoMasConstraint(asConstraint);
#else
        missingIpoptErrorReport();
#endif
    }


    bool InverseKinematics::isCOMAConstraint()
    {
#ifdef IDYNTREE_USES_IPOPT
        return IK_PIMPL(m_pimpl)->isCoMaConstraint();
#else
        return missingIpoptErrorReport();
#endif
    }

    // this should be const reference but we keep it like this to avoid breaking the API. Please change me in iDynTree 4.0
    void InverseKinematics::setCOMTarget(const Position& desiredPosition, double weight)
    {
#ifdef IDYNTREE_USES_IPOPT
        IK_PIMPL(m_pimpl)->setCoMTarget(desiredPosition, weight);
#else
        missingIpoptErrorReport();
#endif
    }

    bool InverseKinematics::setCOMTarget(iDynTree::Span<const double> desiredPosition, double weight)
    {
        constexpr int expected_pos_size = 3;
        bool ok = (desiredPosition.size() == expected_pos_size);
        if( !ok )
        {
            reportError("InverseKinematics","setCOMTarget","Wrong size in input desiredPosition");
            return false;
        }

        this->setCOMTarget(Position(desiredPosition), weight);

        return true;
    }

    void InverseKinematics::setCOMAsConstraintTolerance(double tolerance)
    {
#ifdef IDYNTREE_USES_IPOPT
        IK_PIMPL(m_pimpl)->setCoMasConstraintTolerance(tolerance);
#else
        missingIpoptErrorReport();
#endif
    }


    void InverseKinematics::deactivateCOMTarget()
    {
#ifdef IDYNTREE_USES_IPOPT
        IK_PIMPL(m_pimpl)->setCoMTargetInactive();
#else
        missingIpoptErrorReport();
#endif
    }

    void InverseKinematics::setCOMConstraintProjectionDirection(const iDynTree::Vector3& direction)
    {
        this->setCOMConstraintProjectionDirection(make_span(direction));
    }

    bool InverseKinematics::setCOMConstraintProjectionDirection(iDynTree::Span<const double> direction)
    {
        constexpr int expected_pos_size = 3;
        bool ok = (direction.size() == expected_pos_size);
        if( !ok )
        {
            reportError("InverseKinematics",
                        "setCOMConstraintProjectionDirection",
                        "Wrong size in input direction");
            return false;
        }

#ifdef IDYNTREE_USES_IPOPT
        // define the projection matrix 'Pdirection' in the class 'ConvexHullProjectionConstraint'
        IK_PIMPL(m_pimpl)->m_comHullConstraint_projDirection = direction;
        IK_PIMPL(m_pimpl)->m_comHullConstraint.setProjectionAlongDirection(direction);

        return true;
#else
        return missingIpoptErrorReport();
#endif
    }
}
