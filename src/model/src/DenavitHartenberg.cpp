/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Model/DenavitHartenberg.h>
#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/RevoluteJoint.h>
#include <iDynTree/Model/FixedJoint.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/FreeFloatingState.h>

#include <iDynTree/Core/EigenHelpers.h>


namespace iDynTree
{

void DHChain::setNrOfDOFs(size_t nDofs)
{
    dhParams.resize(nDofs);
    dofNames.resize(nDofs);
}

size_t iDynTree::DHChain::getNrOfDOFs() const
{
    return dhParams.size();
}

void iDynTree::DHChain::setH0(const Transform & _H0)
{
    H0 = _H0;
}

const Transform & DHChain::getH0() const
{
    return H0;
}

void iDynTree::DHChain::setHN(const Transform & _HN)
{
    HN = _HN;
}

const Transform& DHChain::getHN() const
{
    return HN;
}

iDynTree::DHLink& iDynTree::DHChain::operator()(const size_t i)
{
    return dhParams[i];
}

const iDynTree::DHLink& iDynTree::DHChain::operator()(const size_t i) const
{
    return dhParams[i];
}

void DHChain::setDOFName(size_t dofIdx, const std::string &dofName)
{
    assert(dofIdx < dofNames.size());
    dofNames[dofIdx] = dofName;
}

std::string DHChain::getDOFName(size_t dofIdx) const
{
    assert(dofIdx < dofNames.size());
    return dofNames[dofIdx];
}


bool DHChain::fromModel(const Model& model,
                        std::string baseFrame,
                        std::string eeFrame)
{
    return ExtractDHChainFromModel(model,baseFrame,eeFrame,*this);
}

bool DHChain::toModel(Model& outputModel) const
{
    return CreateModelFromDHChain(*this,outputModel);
}



Transform TransformFromDHCraig1989(double a,double alpha,double d,double theta)
{
    // returns Modified Denavit-Hartenberg parameters (According to Craig)
    double ct,st,ca,sa;
    ct = cos(theta);
    st = sin(theta);
    sa = sin(alpha);
    ca = cos(alpha);
    return Transform(Rotation(ct,       -st,     0,
                              st*ca,  ct*ca,   -sa,
                              st*sa,  ct*sa,    ca),
                     Position(a, -sa*d, ca*d));
}

Transform TransformFromDH(double a,double alpha,double d,double theta)
// returns Denavit-Hartenberg parameters (Non-Modified DH)
{
        double ct,st,ca,sa;
        ct = cos(theta);
        st = sin(theta);
        sa = sin(alpha);
        ca = cos(alpha);
        return Transform(Rotation(ct,    -st*ca,   st*sa,
                                  st,     ct*ca,  -ct*sa,
                                   0,        sa,      ca   ),
                         Position(a*ct,      a*st,  d));
}

/**
 * Given two lines, find their closest points (i.e. the points belonging to the common normal)
 *
 * Using the algorithm in http://geomalgorithms.com/a07-_distance.html
 *
 * @param[in]
 * @param[in]

 * @return true if the all went well, false it the lines are parallel.
 */
bool closestPoints(const iDynTree::Axis line_A,
                   const iDynTree::Axis line_B,
                         iDynTree::Position & closest_point_line_A,
                         iDynTree::Position & closest_point_line_B,
                         double tol = 1e-6
                  )
{
    /*
      Using the notation in : http://geomalgorithms.com/a07-_distance.html
      direction_line_A is u
      origin_line_A is P_0
      direction_line_B is v
      origin_line_b is Q_0
      closest_point_line_A is P_C
      closest_point_line_B is Q_C
    */


    Eigen::Vector3d origin_line_A = toEigen(line_A.getOrigin());
    Eigen::Vector3d origin_line_B = toEigen(line_B.getOrigin());
    Eigen::Vector3d direction_line_A = toEigen(line_A.getDirection());
    Eigen::Vector3d direction_line_B = toEigen(line_B.getDirection());

    Eigen::Vector3d w0 = origin_line_A-origin_line_B;
    double a = direction_line_A.dot(direction_line_A);
    double b = direction_line_A.dot(direction_line_B);
    double c = direction_line_B.dot(direction_line_B);
    double d = direction_line_A.dot(w0);
    double e = direction_line_B.dot(w0);

    double denominator = a*c-b*b;

    // test with fabs because sometimes the numerical zero is computed as -epsilon
    if( fabs(denominator) < tol )
    {
        return false;
    }

    //Denominator should be nonnegative
    assert(denominator >= 0.0);

    double s_C = (b*e-c*d)/denominator;
    double t_C = (a*e-b*d)/denominator;

    toEigen(closest_point_line_A) = origin_line_A + s_C*direction_line_A;
    toEigen(closest_point_line_B) = origin_line_B + t_C*direction_line_B;

    return true;
}

/**
 * Check if two axes are incident (i.e. they share at least a point).
 */
bool checkIfAxesAreIncident(const iDynTree::Axis line_A,
                            const iDynTree::Axis line_B,
                                  double tol=1e-6)
{
    Position closest_point_line_A, closest_point_line_B;
    bool areAxesNotParallel = closestPoints(line_A, line_B, closest_point_line_A, closest_point_line_B);

    if (areAxesNotParallel)
    {
        bool arePointCoincident = ((toEigen(closest_point_line_A)-toEigen(closest_point_line_B)).norm() < tol);

        return arePointCoincident;
    }
    else
    {
        // If the axes are parallel, the axes are incident if they are coincident
        // In particular, they are coincident either if the origin are coincident
        // or if the difference between the origin is parallel as well
        bool areOriginCoincident = ((toEigen(line_A.getOrigin())-toEigen(line_B.getOrigin())).norm() < tol);

        if (areOriginCoincident)
        {
            return true;
        }

        Direction originDiffDirection;
        toEigen(originDiffDirection) = (toEigen(line_A.getOrigin())-toEigen(line_B.getOrigin())).normalized();

        bool areAxisCoincident = originDiffDirection.isParallel(line_A.getDirection(),tol);

        return areAxisCoincident;
    }
}

/**
 * Check if two axes are coincident (i.e. they contains the same points).
 */
bool checkIfAxesAreCoincident(const iDynTree::Axis line_A,
                              const iDynTree::Axis link_B,
                                  double tol=1e-6)
{
    bool areAxesIncident = checkIfAxesAreIncident(line_A, link_B, tol);
    bool areDirectionParallel = line_A.getDirection().isParallel(link_B.getDirection(), tol);
    return areAxesIncident && areDirectionParallel;
}

/**
 * Given two lines, find the dh parameters that describe  the transformation between the two axis
 *                  and return the origin and the x,y axis of the DH reference frame
 *                  (run step 3,4,5,7 of section 3.2.3 of
 *                       http://www.cs.duke.edu/brd/Teaching/Bio/asmb/current/Papers/chap3-forward-kinematics.pdf )
 *                  The input lines and the output origin are expressed in the same frame (an inertial frame)
 *
 * The _hint variable are necessary when the DH convention allow freedom of choice in choosing dh_direction_axis_x_n or
 *   dh_origin_n, to preserve injectivity.
 *
 * @param[in] zAxis_i_minus_1 axis of the i-1 joint, given by the chain structure.
 * @param[in] xAxis_i_minus_1 x axis of the i-1 frame, given by a previous call to calculateDH.
 * @param[in] origin_i_minus_1 origin of the i-1 frame, given by a previous call to calculateDH.
 * @param[in] zAxis_i axis of the i joint, given by the chain structure.
 * @param[out] origin_i origin of the i frame, computed by this function.
 * @param[out] xAxis_i x axis of the i-th frame, output of this function.
 * @param[out] yAxis_i y axis of the i-th frame, output of this function.
 * @param[out] dhParams dh params of i-th frame, output of this function.
 */
bool calculateDH(const iDynTree::Axis zAxis_i_minus_1,
                 const iDynTree::Axis xAxis_i_minus_1,
                 const iDynTree::Position origin_i_minus_1,
                 const iDynTree::Axis zAxis_i,
                 const iDynTree::Direction xAxis_n_direction_hint,
                       iDynTree::Position & origin_i,
                       iDynTree::Axis & xAxis_i,
                       iDynTree::Axis & yAxis_i,
                       DHLink & dhParams,
                       double tol = 1e-6,
                       int verbose = 0)
{
    // STEP 3 : Locate O_i (the origin of i-th frame)
    //          Locate the origin O_i where the common normal to z_i and z_{i-1} intersects
    //          z_i . If z_i intersects z_{i-1}, locate O_i at this intersection. If z_i and
    //          z_{i-1} are parallel, locate O_i to any convenenient location along z_i .
    bool zAxes_not_parallel;
    iDynTree::Position point_on_zAxis_i_minus_1_closest_to_zAxis_i;

    // Compute the closest point of the axis z_i-1 and z_i
    // During this computation we get as a byproduct if the two
    // axis are parallel. If the axis are not parallel, then the
    // origin of the frame i is given by the point on z_i closest
    // to z_i_minus_1
    zAxes_not_parallel = closestPoints(zAxis_i_minus_1,
                                 zAxis_i,
                                 point_on_zAxis_i_minus_1_closest_to_zAxis_i,
                                 origin_i,
                                 tol);

    if( !zAxes_not_parallel )
    {
        //if parallel, the origin is not specified and we resort to the original one of the chain
        origin_i = zAxis_i.getOrigin();
    }

    // At this point, we can already set the origin of the the axis of i-th frame

    // \todo check that origin_i actually lies on zAxis_i

    xAxis_i.setOrigin(origin_i);
    yAxis_i.setOrigin(origin_i);

    //
    // We check if z_i and z_i-1 are incident
    //
    bool zAxes_incident = checkIfAxesAreIncident(zAxis_i_minus_1,zAxis_i);

    bool zAxes_coincident = checkIfAxesAreCoincident(zAxis_i_minus_1,zAxis_i);

    //STEP 4 : Establish the direction of x axis of the i-th frame
    //         Establish x_i along the common normal between z_{i-1} and z_i throught O_i,
    //         or in the direction normal to the z_{i-1} - z_{i} plane if z_{i-1} and z_i intersect.

    // While the book distingush just two cases for STEP 4 , we need to actually distinguish three
    // different cases :
    // * z_{i-1} and z_{i} are not incident : in this case the direction of the common normal is always defined,
    //   and in particular we compute it as the difference between the two origins, projected on the spaces
    //   orthogonal to z_i.
    // * z_{i-1} and z_{i} are incident, but not coincident: then the two axis define a plane with a unique axis
    // * z_{i-1} and z_{i} are coincident (i.e. incident and parallel): in this case there are infinite normals,
    //   and so we choose x_i using the hint provided (tipically the x_i used in the previous representation).
    if( !zAxes_incident )
    {
        // not incident

        // If the axis are not incident, the x axis is the common normal of the two axis
        Eigen::Vector3d origin_diff = toEigen(origin_i-origin_i_minus_1);

        // To actually get the normal, we remove from the origin_diff the projection on z_i_minus_1
        Eigen::Vector3d x_i_candidate_eig = origin_diff-origin_diff.dot(toEigen(zAxis_i_minus_1.getDirection()))*toEigen(zAxis_i_minus_1.getDirection());

        x_i_candidate_eig.normalize();

        iDynTree::Direction x_i_candidate;
        iDynTree::toEigen(x_i_candidate) = x_i_candidate_eig;

        xAxis_i.setDirection(x_i_candidate);
    }
    else
    {
        if( !zAxes_coincident )
        {
            // incident but not coincident
            Eigen::Vector3d x_i_candidate_eig  = toEigen(zAxis_i_minus_1.getDirection()).cross(toEigen(zAxis_i.getDirection()));
            x_i_candidate_eig.normalize();

            // Check direction, use hint
            //The positive direction of the axis_x_n is arbitrary, however for dealing with limit case (link where
           // only alpha is different from zero)
           double dh_direction_axis_x_n_sign = x_i_candidate_eig.dot(toEigen(xAxis_n_direction_hint) );
           if (dh_direction_axis_x_n_sign < 0.0)
           {
               x_i_candidate_eig = -x_i_candidate_eig;
           }

           iDynTree::Direction x_i_candidate;
           iDynTree::toEigen(x_i_candidate) = x_i_candidate_eig;

           xAxis_i.setDirection(x_i_candidate);

           std::cerr << "ExtractDHChainFromModel : DH representation is not unique, made an arbitrary choice for x_i sign\n";
        }
        else
        {
            // coincident
            // if the two axis are coincident, the direction of dh_direction_axis_x_n is totally arbitrary
            // as long as it is perpendicular to zAxis. We will take then the hint provided by : direction_axis_x_n_hint
            // to get the dh_direction_axis_x_n, we will project direction_axis_x_n_hint onto the plane
            // perpendicular to axis_z_n_minus_1 == axis_z_n, and will normalize the resulting vector
            // if dh_direction_axis_x_n is parallel to the two axis, we just apply the same procedure to (1,0,0), (0,1,0)
            // and (0,0,1) (in the base frame) in this order. At least two of this three vectors will not be parallel with
            // the axis of the joint, and so they can be used as hints for the direction of
            std::vector<iDynTree::Direction> candidateDirectionHints;
            candidateDirectionHints.push_back(xAxis_n_direction_hint);
            candidateDirectionHints.push_back(iDynTree::Direction(1.0, 0.0, 0.0));
            candidateDirectionHints.push_back(iDynTree::Direction(0.0, 1.0, 0.0));
            candidateDirectionHints.push_back(iDynTree::Direction(0.0, 0.0, 1.0));

            iDynTree::Direction xDirectionHintNotParallelToZ;
            for(int k=0; k < candidateDirectionHints.size(); k++)
            {
                if (candidateDirectionHints[k].isParallel(zAxis_i.getDirection(), tol))
                {
                    continue;
                }
                else
                {
                    xDirectionHintNotParallelToZ = candidateDirectionHints[k];
                    break;
                }
            }

            Eigen::Vector3d direction_axis_z_i_eig = toEigen(zAxis_i.getDirection());
            Eigen::Vector3d x_i_candidate_eig = toEigen(xDirectionHintNotParallelToZ) -
                                                toEigen(xDirectionHintNotParallelToZ).dot(direction_axis_z_i_eig) *
                                                direction_axis_z_i_eig;

            x_i_candidate_eig.normalize();

            iDynTree::Direction x_i_candidate;
            iDynTree::toEigen(x_i_candidate) = x_i_candidate_eig;

            xAxis_i.setDirection(x_i_candidate);

            std::cerr << "ExtractDHChainFromModel : DH representation is not unique, made an arbitrary choice for x_i direction\n";
        }
    }

    // STEP 4 ADDENDUM : get y_i

    // Once the direction of axis z_n and x_n has been determined,
    //  the direction of axis y_n is simply given by a cross product to
    //  ensure a right handed coordinates system
    iDynTree::Direction y_i_candidate;
    toEigen(y_i_candidate) = toEigen(zAxis_i.getDirection()).cross(toEigen(xAxis_i.getDirection()));

    yAxis_i.setDirection(y_i_candidate);


    ////////////////////////////////////////////////////////////////////
    // STEP 5: Computation of DH parameters
    ////////////////////////////////////////////////////////////////////

    //calculation of a_i
    //distance along x_i from O_i to the intersection of the x_i and z_{i-1} axes
    iDynTree::Position x_i_z_i_minus_1_intersection_A, x_i_z_i_minus_1_intersection_B;

    closestPoints(zAxis_i_minus_1,
                  xAxis_i,
                  x_i_z_i_minus_1_intersection_A,
                  x_i_z_i_minus_1_intersection_B,
                  tol);

    //x_i and z_{i-1} should intersecate
    assert(checkIfAxesAreIncident(xAxis_i,zAxis_i_minus_1));

    dhParams.A = -(toEigen(x_i_z_i_minus_1_intersection_B)-toEigen(origin_i)).dot(toEigen(xAxis_i.getDirection()));

    //calculation of d_i
    //distance along z_{i-1} from O_{i-1} to the intersection of the x_i and z_{i-1} axes
    dhParams.D = (toEigen(x_i_z_i_minus_1_intersection_A)-toEigen(origin_i_minus_1)).dot(toEigen(zAxis_i_minus_1.getDirection()));

    //calculation of alpha_i
    //angle between z_{i-1} and z_i measured about x_i
    double cos_alpha_i = toEigen(zAxis_i_minus_1.getDirection()).dot(toEigen(zAxis_i.getDirection()));
    //assert(((direction_axis_z_n_minus_1*direction_axis_z_n)*dh_direction_axis_x_n).Norm() < tol);
    double sin_alpha_i = (toEigen(zAxis_i_minus_1.getDirection()).cross(toEigen(zAxis_i.getDirection()))).dot(toEigen(xAxis_i.getDirection()));
    //assert( fabs(cos_alpha_i*cos_alpha_i + sin_alpha_i*sin_alpha_i - 1) < tol);

    dhParams.Alpha = atan2(sin_alpha_i,cos_alpha_i);

    //calculation of theta_i
    //angle between x_{i-1} and x_i measure about z_{i-1}
    double cos_theta_i = toEigen(xAxis_i_minus_1.getDirection()).dot(toEigen(xAxis_i.getDirection()));
    double sin_theta_i = (toEigen(xAxis_i_minus_1.getDirection()).cross(toEigen(xAxis_i.getDirection()))).dot(toEigen(zAxis_i_minus_1.getDirection()));
    dhParams.Offset = atan2(sin_theta_i,cos_theta_i);

    return true;
}


bool transformFromAxes(const Axis xAxis,
                       const Axis yAxis,
                       const Axis zAxis,
                             Transform& refFrame_H_frame,
                             double tol=1e-5)
{
    // Check axis are consistent (they share the origin)
    Position xyIntersectionOnX, xyIntersectionOnY, yzIntersectionOnY, yzIntersectionOnZ;
    bool ok = closestPoints(xAxis, yAxis, xyIntersectionOnX, xyIntersectionOnY, tol);
    if (!ok)
    {
        reportError("","transformFromAxes","xAxis is parallel with the yAxis");
        return false;
    }

    ok = closestPoints(yAxis, zAxis, yzIntersectionOnY, yzIntersectionOnZ, tol);
    if (!ok)
    {
        reportError("","transformFromAxes","yAxis is parallel with the zAxis");
        return false;
    }

    if ((toEigen(xyIntersectionOnX)-toEigen(xyIntersectionOnY)).norm() > tol)
    {
        reportError("","transformFromAxes","x and y are not incident");
        return false;
    }

    if ((toEigen(yzIntersectionOnY)-toEigen(yzIntersectionOnZ)).norm() > tol)
    {
        reportError("","transformFromAxes","x and y are not incident");
        return false;
    }

    if ((toEigen(xyIntersectionOnX)-toEigen(yzIntersectionOnY)).norm() > tol)
    {
        reportError("","transformFromAxes","xy intersection is not coincident with yz intersection");
        return false;
    }

    iDynTree::Direction zDirectionCheck;
    toEigen(zDirectionCheck) = toEigen(xAxis.getDirection()).cross(toEigen(yAxis.getDirection()));
    if (!zDirectionCheck.isParallel(zAxis.getDirection(),tol))
    {
        reportError("","transformFromAxes","x cross y is not parallel to z");
        return false;
    }

    if ((toEigen(zDirectionCheck).dot(toEigen(zAxis.getDirection()))) < 0.0)
    {
        reportError("","transformFromAxes","x cross y is not in the same direction of z");
        return false;
    }

    refFrame_H_frame.setPosition(xyIntersectionOnX);
    iDynTree::Rotation rot;
    toEigen(rot).block<3,1>(0,0) = toEigen(xAxis.getDirection());
    toEigen(rot).block<3,1>(0,1) = toEigen(yAxis.getDirection());
    toEigen(rot).block<3,1>(0,2) = toEigen(zAxis.getDirection());
    refFrame_H_frame.setRotation(rot);

    return true;
}

bool ExtractDHChainFromModel(const Model& model,
                             const std::string baseFrame,
                             const std::string eeFrame,
                                   DHChain& outputChain,
                                   double tolerance)
{
    bool globalOk = true;

    // We check that the frame exist
    if( model.getFrameIndex(baseFrame) == iDynTree::FRAME_INVALID_INDEX ||
        model.getFrameIndex(eeFrame) == iDynTree::FRAME_INVALID_INDEX )
    {
        std::string err = "Frame " + baseFrame + " or frame " + eeFrame + " not found in model.";
        reportError("","ExtractDHChainFromModel",err.c_str());
        return false;
    }

    LinkIndex linkOfBaseFrameIdx = model.getFrameLink(model.getFrameIndex(baseFrame));
    LinkIndex linkOfEEFrameIdx = model.getFrameLink(model.getFrameIndex(eeFrame));

    // To extract the DH parameters, we need first to express all the
    // axis of the chain in a common frame, to simplify the computations.
    // As a first step, we compute the traversal that has as base link
    // the link at which the base frame is attached
    Traversal traversal;
    model.computeFullTreeTraversal(traversal,linkOfBaseFrameIdx);

    // We then loop from the eeLink to the baseLink, counting
    // the number of revolute joints in the middle, giving an error if
    // there is a non-revolute or non-fixed joint
    size_t nrOfDofs = 0;
    for(LinkIndex visitedLinkIdx = linkOfEEFrameIdx;
        visitedLinkIdx != linkOfBaseFrameIdx;
        visitedLinkIdx = traversal.getParentLinkFromLinkIndex(visitedLinkIdx)->getIndex())
    {
        IJointConstPtr joint = traversal.getParentJointFromLinkIndex(visitedLinkIdx);
        bool isRevoluteJoint =
            ( dynamic_cast<const RevoluteJoint*>(joint) != 0 );
        bool isFixedJoint    =
            ( dynamic_cast<const FixedJoint*>(joint) != 0 );

        if( !isFixedJoint && !isRevoluteJoint )
        {
            std::string err = "Joint  " + model.getJointName(joint->getIndex()) + " is not revolute neither fixed, but the DH converter only support this two joints.";
            reportError("","ExtractDHChainFromModel",err.c_str());
        }

        if( isRevoluteJoint )
        {
            nrOfDofs++;
        }
    }

    // Now we know that the output DH chain will have nrOfDofs dofs
    outputChain.setNrOfDOFs(nrOfDofs);

    // We need to write all the revolution axis in the same frame, to simplify computations.
    // The first step is to compute the link position with respect to the base frame, that
    // we can do with the usual fwd pos kinematics.

    FreeFloatingPos chainPos(model);
    // We use the baseFrame as "world", to easily compute the baseFrame_X_link transform
    // using the usual floating base forward kinematics.
    chainPos.worldBasePos() = model.getFrameTransform(model.getFrameIndex(baseFrame)).inverse();
    chainPos.jointPos().zero();
    LinkPositions baseFrame_X_link(model);
    bool ok = ForwardPositionKinematics(model,traversal,chainPos,baseFrame_X_link);

    if( !ok )
    {
        reportError("","ExtractDHChainFromModel","Error in computing ForwardKinematics.");
        return false;
    }

    // Step 1: Locate and label the joint axes z_1 , . . . , z_{nrOfDofs} .
    // For simplicity, we express all the axes in the base frame
    size_t nrOFDHFrames = nrOfDofs+1;
    std::vector<iDynTree::Axis> zAxes(nrOFDHFrames);
    std::vector<iDynTree::Axis> xAxes(nrOFDHFrames);
    std::vector<iDynTree::Axis> yAxes(nrOFDHFrames);
    std::vector<iDynTree::Position> dh_origin(nrOFDHFrames);

    // If we count the joints as joint 1 , joint 2 ... joint nrOfDofs,
    // the zAxis of frame i is associated with the joint i+1
    size_t counter = nrOfDofs;
    for(LinkIndex visitedLinkIdx = linkOfEEFrameIdx;
        visitedLinkIdx != linkOfBaseFrameIdx;
        visitedLinkIdx = traversal.getParentLinkFromLinkIndex(visitedLinkIdx)->getIndex())
    {
        IJointConstPtr joint = traversal.getParentJointFromLinkIndex(visitedLinkIdx);
        std::string jointName = model.getJointName(joint->getIndex());
        bool isRevoluteJoint =
            ( dynamic_cast<const RevoluteJoint*>(joint) != 0 );

        if( isRevoluteJoint )
        {
            const RevoluteJoint * revJoint = dynamic_cast<const RevoluteJoint*>(joint);
            zAxes[counter-1] = baseFrame_X_link(visitedLinkIdx)*revJoint->getAxis(visitedLinkIdx);
            outputChain.setDOFName(counter-1, jointName);
            if (revJoint->hasPosLimits())
            {
                outputChain(counter-1).Max = revJoint->getMaxPosLimit(0);
                outputChain(counter-1).Min = revJoint->getMinPosLimit(0);
            }
            else
            {
                outputChain(counter-1).Max =  std::numeric_limits<double>::infinity();
                outputChain(counter-1).Min = -std::numeric_limits<double>::infinity();
            }
            counter--;
        }
    }

    assert(counter == 0);

    // Step 7 : the z_{nrOfDofs} is the zAxis of the end effector frame
    Transform base_H_endEffector = baseFrame_X_link(linkOfEEFrameIdx)*model.getFrameTransform(model.getFrameIndex(eeFrame));
    zAxes[nrOfDofs] = base_H_endEffector*Axis(Direction(0,0,1.0),Position::Zero());

    // We have the H0 transformation to account for the
    // transform between the base frame and the first frame (0)
    // of the DH representation. Hence, we have to choose the
    // first frame for the DH representation. We already have its origin
    // and the z axis, we have then to choose an arbitrary direction for the
    // x and y axis.
    // The strategy that we use for getting this x and y axes is the following:
    //      * if the z axis of the base frame and the 0 DH frame are
    //        parallel, we can assign the x and y axes of the 0 DH frame
    //        to be the same x and y axes of the base frame.
    //      * if the z axis of the base frame and the 0 DH frame are
    //        not parallel, we can find a rotation around an axis that is
    //        transforming the z axis of the base frame to the 0 frame, and
    //        we apply the same transformation to the x and y axes of the base frame
    //        to obtain the x and y axes fo the DH frame.

    // We write the z axis in the base frame
    Axis z_in_base = Axis(Direction(0,0,1.0),Position::Zero());

    // We check if z_in_base and the joint axes of the first joint are parallel
    bool z_base_and_z_dh_0_are_parallel = z_in_base.isParallel(zAxes[0],tolerance);

    Direction xDirection0;
    Direction yDirection0;

    if( z_base_and_z_dh_0_are_parallel )
    {
        xDirection0  = Direction(1,0,0);
        toEigen(yDirection0) = toEigen(zAxes[0].getDirection()).cross(toEigen(xDirection0));
        yDirection0.Normalize();
    }
    else
    {
        iDynTree::Direction rotAxis;
        Eigen::Vector3d nonNormalizedRotAxis = toEigen(Direction(0,0,1.0)).cross(toEigen(zAxes[0].getDirection()));
        toEigen(rotAxis) = nonNormalizedRotAxis;
        rotAxis.Normalize();
        double rot_angle = atan2(toEigen(rotAxis).dot(nonNormalizedRotAxis),toEigen(Direction(0,0,1.0)).dot(toEigen(zAxes[0].getDirection())));
        iDynTree::Rotation base_R_0 = iDynTree::Rotation::RotAxis(rotAxis,rot_angle);
        xDirection0 = base_R_0*Direction(1,0,0);
        yDirection0 = base_R_0*Direction(0,1,0);
    }

    assert(xDirection0.isPerpendicular(zAxes[0].getDirection(),tolerance));
    assert(yDirection0.isPerpendicular(zAxes[0].getDirection(),tolerance));

    // We must select the origin, the x and y axis of dh_0 frame so we can initialize the recursive procedure that identifies
    // the origin, x and y of each dh frame (the z axis are already determined by the axes directions)
    dh_origin[0] = zAxes[0].getOrigin();
    xAxes[0].setOrigin(dh_origin[0]);
    xAxes[0].setDirection(xDirection0);
    yAxes[0].setOrigin(dh_origin[0]);
    yAxes[0].setDirection(yDirection0);

    // We actually compute the DH parameters
    // and select the origin, x and y axis of the DH frames
    for(size_t i=0; i < nrOfDofs; i++ )
    {
        DHLink dhLink = outputChain(i);
        bool ok = calculateDH(zAxes[i],
                              xAxes[i],
                              dh_origin[i],
                              zAxes[i+1],
                              Direction(1.0,0.0,0.0),
                              dh_origin[i+1],
                              xAxes[i+1],
                              yAxes[i+1],
                              dhLink);
        outputChain(i) = dhLink;


        if (!ok)
        {
            reportError("","ExtractDHChainFromModel","Error in extracing dh parameters for one link of the chain");
        }

        globalOk = globalOk && ok;
    }

    // transform matrix from the 0th frame to the base frame
    Transform base_H_0;
    ok = transformFromAxes(xAxes[0],yAxes[0],zAxes[0],base_H_0);
    globalOk = globalOk && ok;

    outputChain.setH0(base_H_0);

    //transform from the end-effector to the N-th frame
    Transform base_H_N;
    ok = transformFromAxes(xAxes[nrOfDofs],
                           yAxes[nrOfDofs],
                           zAxes[nrOfDofs],
                           base_H_N);
    globalOk = globalOk && ok;

    Transform N_H_endEffector = base_H_N.inverse()*base_H_endEffector;
    outputChain.setHN(N_H_endEffector);

    return globalOk;
}

std::string inline intToString(const int inInt)
{
    std::stringstream ss;
    ss << inInt;
    return ss.str();
}

bool CreateModelFromDHChain(const DHChain& inputChain,
                                  Model& outputModel)
{
    // First we clear the model
    outputModel = Model();

    // All the inertial will be of one kg in the origin
    iDynTree::SpatialInertia inertiaDefault(1.0,Position::Zero(),RotationalInertiaRaw::Zero());
    iDynTree::Link linkDefault;
    linkDefault.setInertia(inertiaDefault);

    std::string baseLinkName  = "link0";


    // Then we create a base link
    outputModel.addLink(baseLinkName,linkDefault);

    std::string previousLinkName = baseLinkName;

    for(int i=0; i<inputChain.getNrOfDOFs(); i++)
    {
        std::string addedJointName = inputChain.getDOFName(i);
        std::string addedLinkName = "link"+intToString(i+1);

        Transform parent_H_child;
        Axis rot_axis_in_parent_frame;
        Axis axisOnZThroughOrigin = Axis(Direction(0.0,0.0,1.0),Position::Zero());
        Transform dhTransform = TransformFromDH(inputChain(i).A,inputChain(i).Alpha,inputChain(i).D,inputChain(i).Offset);
        parent_H_child = dhTransform;
        rot_axis_in_parent_frame = axisOnZThroughOrigin;

        RevoluteJoint addedJoint = RevoluteJoint(parent_H_child,rot_axis_in_parent_frame);
        outputModel.addJointAndLink(previousLinkName,
                                    addedJointName,&(addedJoint),
                                    addedLinkName,linkDefault);

        previousLinkName = addedLinkName;
    }

    // Add base and end effector additional frames, using the H0 and HN information
    outputModel.addAdditionalFrameToLink(baseLinkName,"baseFrame",inputChain.getH0().inverse());
    outputModel.addAdditionalFrameToLink("link"+intToString(inputChain.getNrOfDOFs()),"distalFrame",inputChain.getHN());

    return true;
}



}