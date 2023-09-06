// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_DENAVIT_HARTENBERG_H
#define IDYNTREE_DENAVIT_HARTENBERG_H

#include <iDynTree/Indices.h>
#include <iDynTree/Model.h>

#include <iDynTree/Transform.h>

#include <vector>

namespace iDynTree
{

/**
 * Structure representing
 * the four DH parameters of
 * a link in a Chain.
 */
struct DHLink
{
   double A;
   double D;
   double Alpha;
   double Offset;
   double Min;
   double Max;
};

/**
 * Simple representation of a chain described with
 * Denavit-Hartenberg Parameters.
 * Directly inspiered by the iCub::iKin::iKinChain class.
 */
class DHChain
{
private:
    Transform H0;
    std::vector<DHLink> dhParams;
    Transform HN;
    std::vector<std::string> dofNames;

public:
    void   setNrOfDOFs(size_t nDofs);
    size_t getNrOfDOFs() const;

    void setH0(const iDynTree::Transform & _H0);
    const iDynTree::Transform & getH0() const;

    void setHN(const Transform & _HN);
    const Transform & getHN() const;

    /**
     * Return a reference to the i-th link of the chain.
     */
    DHLink & operator() (const size_t i);

    /**
     * Return a reference to the i-th link of the chain (const version).
     */
    const DHLink & operator() (const size_t i) const;

    /**
     * Get the name of the i-th DOF.
     */
    std::string getDOFName(size_t dofIdx) const;

    /**
     * Get the name of the i-th DOF.
     */
    void setDOFName(size_t dofIdx, const std::string &dofName);

    /**
     * Create an iDynTree::Model from this DHChain.
     * \see CreateModelFromDHChain .
     */
    bool toModel(Model & outputModel) const;

    /**
     * Create a DHChain from a Model.
     * \see ExtractDHChainFromModel .
     */
    bool fromModel(const iDynTree::Model & model,
                   std::string baseFrame,
                   std::string eeFrame);
};

/*
 * DH_Craig1989 : constructs a transformationmatrix
 * T_link(i-1)_link(i) with the Denavit-Hartenberg convention as
 * described in the Craigs book: Craig, J. J.,Introduction to
 * Robotics: Mechanics and Control, Addison-Wesley,
 * isbn:0-201-10326-5, 1986.
 *
 * Note that the frame is a redundant way to express the information
 * in the DH-convention.
 * \verbatim
 * Parameters in full : a(i-1),alpha(i-1),d(i),theta(i)
 *
 *  axis i-1 is connected by link i-1 to axis i numbering axis 1
 *  to axis n link 0 (immobile base) to link n
 *
 *  link length a(i-1) length of the mutual perpendicular line
 *  (normal) between the 2 axes.  This normal runs from (i-1) to
 *  (i) axis.
 *
 *  link twist alpha(i-1): construct plane perpendicular to the
 *  normal project axis(i-1) and axis(i) into plane angle from
 *  (i-1) to (i) measured in the direction of the normal
 *
 *  link offset d(i) signed distance between normal (i-1) to (i)
 *  and normal (i) to (i+1) along axis i joint angle theta(i)
 *  signed angle between normal (i-1) to (i) and normal (i) to
 *  (i+1) along axis i
 *
 *   First and last joints : a(0)= a(n) = 0
 *   alpha(0) = alpha(n) = 0
 *
 *   PRISMATIC : theta(1) = 0 d(1) arbitrarily
 *
 *   REVOLUTE : theta(1) arbitrarily d(1) = 0
 *
 *   Not unique : if intersecting joint axis 2 choices for normal
 *   Frame assignment of the DH convention : Z(i-1) follows axis
 *   (i-1) X(i-1) is the normal between axis(i-1) and axis(i)
 *   Y(i-1) follows out of Z(i-1) and X(i-1)
 *
 *     a(i-1)     = distance from Z(i-1) to Z(i) along X(i-1)
 *     alpha(i-1) = angle between Z(i-1) to Z(i) along X(i-1)
 *     d(i)       = distance from X(i-1) to X(i) along Z(i)
 *     theta(i)   = angle between X(i-1) to X(i) along X(i)
 * \endverbatim
 *
 * Function compatible with KDL's Frame::DH_Craig1989 method.
 */
Transform TransformFromDHCraig1989(double a,double alpha,double d,double theta);

/**
 * DH : constructs a transformationmatrix T_link(i-1)_link(i) with
 * the Denavit-Hartenberg convention as described in the original
 * publictation: Denavit, J. and Hartenberg, R. S., A kinematic
 * notation for lower-pair mechanisms based on matrices, ASME
 * Journal of Applied Mechanics, 23:215-221, 1955.
 *
 * Function compatible on KDL's Frame::DH method.
 */
Transform TransformFromDH(double a,double alpha,double d,double theta);

/**
 * Extract a Denavit Hartenberg Chain from a iDynTree::Model.
 * In particular you can specify the base frame and the end effector frame
 * of the chain. The chain is then extracted using the algorithm found
 * in:
 * Chapter 3, Spong, Mark W., Seth Hutchinson, and M. Vidyasagar. "Robot modeling and control". 2006.
 * (section 3.2.3 of http://www.cs.duke.edu/brd/Teaching/Bio/asmb/current/Papers/chap3-forward-kinematics.pdf)
 *
 *  \note The DH representation supports only revolute and translational
 *        1-DOF joints. In this implementation only revolute joints are supported.
 */
bool ExtractDHChainFromModel(const iDynTree::Model & model,
                             const std::string baseFrame,
                             const std::string eeFrame,
                                   DHChain & outputChain,
                                   double tolerance = 1e-5);

/**
 * Create an iDynTree::Model from a DHChain.
 *
 * \note The names of the links will be link0, link1, ... linkN,
 *       furthermore there are two additional frame for the base and endEffector frames,
 *       named baseFrame and distalFrame .
 * \note The inertia of the links will be set to 1 kg in the origin of the link.
 *
 * @return true if all went ok, false otherwise.
 */
bool CreateModelFromDHChain(const DHChain & inputChain,
                                  Model   & outputModel);


}


#endif
