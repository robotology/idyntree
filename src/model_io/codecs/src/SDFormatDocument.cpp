// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "SDFormatDocument.h"

#include <iDynTree/AccelerometerSensor.h>
#include <iDynTree/FixedJoint.h>
#include <iDynTree/GyroscopeSensor.h>
#include <iDynTree/Link.h>
#include <iDynTree/Model.h>
#include <iDynTree/ModelTransformers.h>
#include <iDynTree/PrismaticJoint.h>
#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/SolidShapes.h>
#include <iDynTree/Utils.h>
#include <iDynTree/VectorFixSize.h>

#ifdef IDYNTREE_USES_SDFORMAT
#include <gz/math/Inertial.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <sdf/sdf.hh>
#endif

#include <algorithm>
#include <string>

namespace iDynTree
{

#ifdef IDYNTREE_USES_SDFORMAT
    namespace
    {

        // Helper function to convert gz::math::Pose3d to iDynTree::Transform
        iDynTree::Transform poseToTransform(const gz::math::Pose3d &pose)
        {
            iDynTree::Transform transform;

            // Convert rotation from quaternion to rotation matrix
            const gz::math::Quaterniond &quat = pose.Rot();

            // iDynTree quaternion format is (w, x, y, z)
            iDynTree::Vector4 quatVec;
            quatVec(0) = quat.W();
            quatVec(1) = quat.X();
            quatVec(2) = quat.Y();
            quatVec(3) = quat.Z();

            iDynTree::Rotation rotation = iDynTree::Rotation::RotationFromQuaternion(quatVec);

            // Set translation
            iDynTree::Position position(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

            transform.setRotation(rotation);
            transform.setPosition(position);

            return transform;
        }

        // Helper function to convert SDF geometry to iDynTree SolidShape
        iDynTree::SolidShape *convertGeometry(const sdf::Geometry *geom)
        {
            if (!geom)
            {
                return nullptr;
            }

            switch (geom->Type())
            {
            case sdf::GeometryType::BOX:
            {
                const sdf::Box *box = geom->BoxShape();
                if (box)
                {
                    iDynTree::Box *idynBox = new iDynTree::Box();
                    gz::math::Vector3d size = box->Size();
                    idynBox->setX(size.X());
                    idynBox->setY(size.Y());
                    idynBox->setZ(size.Z());
                    return idynBox;
                }
                break;
            }
            case sdf::GeometryType::SPHERE:
            {
                const sdf::Sphere *sphere = geom->SphereShape();
                if (sphere)
                {
                    iDynTree::Sphere *idynSphere = new iDynTree::Sphere();
                    idynSphere->setRadius(sphere->Radius());
                    return idynSphere;
                }
                break;
            }
            case sdf::GeometryType::CYLINDER:
            {
                const sdf::Cylinder *cylinder = geom->CylinderShape();
                if (cylinder)
                {
                    iDynTree::Cylinder *idynCylinder = new iDynTree::Cylinder();
                    idynCylinder->setRadius(cylinder->Radius());
                    idynCylinder->setLength(cylinder->Length());
                    return idynCylinder;
                }
                break;
            }
            case sdf::GeometryType::MESH:
            {
                const sdf::Mesh *mesh = geom->MeshShape();
                if (mesh)
                {
                    iDynTree::ExternalMesh *idynMesh = new iDynTree::ExternalMesh();
                    idynMesh->setFilename(mesh->Uri());
                    gz::math::Vector3d scale = mesh->Scale();
                    iDynTree::Vector3 idynScale;
                    idynScale(0) = scale.X();
                    idynScale(1) = scale.Y();
                    idynScale(2) = scale.Z();
                    idynMesh->setScale(idynScale);
                    return idynMesh;
                }
                break;
            }
            default:
                // Unsupported geometry type
                break;
            }

            return nullptr;
        }

    }
#endif

    SDFormatDocument::SDFormatDocument(const iDynTree::ModelParserOptions &options)
        : m_options(options) {}

    SDFormatDocument::~SDFormatDocument() {}

    iDynTree::ModelParserOptions &SDFormatDocument::options() { return m_options; }

    const iDynTree::Model &SDFormatDocument::model() const { return m_model; }

    const iDynTree::SensorsList &SDFormatDocument::sensors() const
    {
        return m_model.sensors();
    }

    bool SDFormatDocument::loadFromFile(
        const std::string &filename, const std::vector<std::string> &packageDirs)
    {
#ifdef IDYNTREE_USES_SDFORMAT
        // Load SDF file
        sdf::Root root;
        sdf::Errors errors = root.Load(filename);

        if (!errors.empty())
        {
            std::string errorMsg = "Error loading SDF file: ";
            for (const auto &error : errors)
            {
                errorMsg += error.Message() + "; ";
            }
            reportError("SDFormatDocument", "loadFromFile", errorMsg.c_str());
            return false;
        }

        // Check if we have a model at the root level
        const sdf::Model *sdfModel = root.Model();

        // If no model at root, check if there's a world with models
        if (!sdfModel && root.WorldCount() > 0)
        {
            const sdf::World *world = root.WorldByIndex(0);
            if (world && world->ModelCount() > 0)
            {
                sdfModel = world->ModelByIndex(0);
            }
        }

        if (!sdfModel)
        {
            reportError("SDFormatDocument", "loadFromFile",
                        "No model found in SDF file");
            return false;
        }

        // Store the model pointer for conversion (we need to keep root alive)
        // For now, we'll convert immediately
        m_sdfModel = std::make_shared<sdf::Model>(*sdfModel);

        // Initialize the iDynTree model
        m_model = iDynTree::Model();
        m_model.setPackageDirs(packageDirs);

        // Convert SDF model to iDynTree model
        return convertSDFormatToModel(packageDirs);
#else
        reportError("SDFormatDocument", "loadFromFile",
                    "iDynTree was not compiled with SDFormat support");
        return false;
#endif
    }

    bool SDFormatDocument::loadFromString(
        const std::string &sdfString, const std::vector<std::string> &packageDirs)
    {
#ifdef IDYNTREE_USES_SDFORMAT
        // Load SDF from string
        sdf::Root root;
        sdf::Errors errors = root.LoadSdfString(sdfString);

        if (!errors.empty())
        {
            std::string errorMsg = "Error loading SDF from string: ";
            for (const auto &error : errors)
            {
                errorMsg += error.Message() + "; ";
            }
            reportError("SDFormatDocument", "loadFromString", errorMsg.c_str());
            return false;
        }

        // Check if we have a model at the root level
        const sdf::Model *sdfModel = root.Model();

        // If no model at root, check if there's a world with models
        if (!sdfModel && root.WorldCount() > 0)
        {
            const sdf::World *world = root.WorldByIndex(0);
            if (world && world->ModelCount() > 0)
            {
                sdfModel = world->ModelByIndex(0);
            }
        }

        if (!sdfModel)
        {
            reportError("SDFormatDocument", "loadFromString",
                        "No model found in SDF string");
            return false;
        }

        // Store the model pointer for conversion
        m_sdfModel = std::make_shared<sdf::Model>(*sdfModel);

        // Initialize the iDynTree model
        m_model = iDynTree::Model();
        m_model.setPackageDirs(packageDirs);

        // Convert SDF model to iDynTree model
        return convertSDFormatToModel(packageDirs);
#else
        reportError("SDFormatDocument", "loadFromString",
                    "iDynTree was not compiled with SDFormat support");
        return false;
#endif
    }

    bool SDFormatDocument::convertSDFormatToModel(
        const std::vector<std::string> & /*packageDirs*/)
    {
#ifdef IDYNTREE_USES_SDFORMAT
        if (!m_sdfModel)
        {
            reportError("SDFormatDocument", "convertSDFormatToModel",
                        "No SDF model available for conversion");
            return false;
        }

        // Cast back to sdf::Model
        const sdf::Model *sdfModel =
            static_cast<const sdf::Model *>(m_sdfModel.get());

        // Initialize solid shapes containers
        m_model.visualSolidShapes().resize(sdfModel->LinkCount());
        m_model.collisionSolidShapes().resize(sdfModel->LinkCount());

        // Initialize sensors list
        m_model.sensors() = iDynTree::SensorsList();

        // Add all links to the model
        for (uint64_t linkIdx = 0; linkIdx < sdfModel->LinkCount(); ++linkIdx)
        {
            const sdf::Link *sdfLink = sdfModel->LinkByIndex(linkIdx);
            if (!sdfLink)
            {
                reportError("SDFormatDocument", "convertSDFormatToModel",
                            "Failed to get link from model");
                return false;
            }

            // Create iDynTree link
            iDynTree::Link idynLink;

            // Set inertial properties
            const gz::math::Inertiald &inertial = sdfLink->Inertial();

            // Get mass
            double mass = inertial.MassMatrix().Mass();

            // Get center of mass position relative to link frame
            const gz::math::Pose3d &inertiaPose = inertial.Pose();
            iDynTree::Position com(inertiaPose.Pos().X(), inertiaPose.Pos().Y(),
                                   inertiaPose.Pos().Z());

            // Get inertia tensor
            // SDFormat's URDF->SDF conversion already transforms the inertia to the link frame.
            const gz::math::Matrix3d &I_linkFrame = inertial.Moi();

            iDynTree::RotationalInertia rotInertiaWrtCom_linkFrame;
            rotInertiaWrtCom_linkFrame(0, 0) = I_linkFrame(0, 0);
            rotInertiaWrtCom_linkFrame(0, 1) = I_linkFrame(0, 1);
            rotInertiaWrtCom_linkFrame(0, 2) = I_linkFrame(0, 2);
            rotInertiaWrtCom_linkFrame(1, 0) = I_linkFrame(1, 0);
            rotInertiaWrtCom_linkFrame(1, 1) = I_linkFrame(1, 1);
            rotInertiaWrtCom_linkFrame(1, 2) = I_linkFrame(1, 2);
            rotInertiaWrtCom_linkFrame(2, 0) = I_linkFrame(2, 0);
            rotInertiaWrtCom_linkFrame(2, 1) = I_linkFrame(2, 1);
            rotInertiaWrtCom_linkFrame(2, 2) = I_linkFrame(2, 2);

            // Create spatial inertia using the helper that handles inertia at COM
            iDynTree::SpatialInertia spatialInertia;
            spatialInertia.fromRotationalInertiaWrtCenterOfMass(mass, com, rotInertiaWrtCom);
            idynLink.setInertia(spatialInertia);

            // Add link to model (link name is part of addLink, not setName)
            m_model.addLink(sdfLink->Name(), idynLink);

            LinkIndex addedLinkIndex = m_model.getLinkIndex(sdfLink->Name());

            // Parse sensors attached to this link
            for (uint64_t sensorIdx = 0; sensorIdx < sdfLink->SensorCount();
                 ++sensorIdx)
            {
                const sdf::Sensor *sdfSensor = sdfLink->SensorByIndex(sensorIdx);
                if (!sdfSensor)
                {
                    std::string errMsg = "Failed to get sensor from link " + sdfLink->Name();
                    reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                    return false;
                }

                sdf::SensorType sensorType = sdfSensor->Type();

                // Convert sensor pose to transform
                iDynTree::Transform sensorTransform =
                    poseToTransform(sdfSensor->RawPose());

                if (sensorType == sdf::SensorType::IMU)
                {
                    // IMU sensors provide both acceleration and angular velocity
                    // We'll create both an accelerometer and gyroscope sensor
                    iDynTree::AccelerometerSensor accSensor;
                    accSensor.setName(sdfSensor->Name() + "_acc");
                    accSensor.setParentLink(sdfLink->Name());
                    accSensor.setParentLinkIndex(addedLinkIndex);
                    accSensor.setLinkSensorTransform(sensorTransform);
                    m_model.sensors().addSensor(accSensor);

                    iDynTree::GyroscopeSensor gyroSensor;
                    gyroSensor.setName(sdfSensor->Name() + "_gyro");
                    gyroSensor.setParentLink(sdfLink->Name());
                    gyroSensor.setParentLinkIndex(addedLinkIndex);
                    gyroSensor.setLinkSensorTransform(sensorTransform);
                    m_model.sensors().addSensor(gyroSensor);
                }
                // Note: Force-torque sensors in SDF are typically attached to joints, not
                // links They would need to be handled separately in the joint parsing
                // section
            }

            // Parse visual geometries
            for (uint64_t visualIdx = 0; visualIdx < sdfLink->VisualCount();
                 ++visualIdx)
            {
                const sdf::Visual *visual = sdfLink->VisualByIndex(visualIdx);
                if (!visual)
                {
                    std::string errMsg = "Failed to get visual from link " + sdfLink->Name();
                    reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                    return false;
                }

                if (!visual->Geom())
                {
                    std::string errMsg = "Visual " + visual->Name() + " in link " +
                                         sdfLink->Name() + " has no geometry";
                    reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                    return false;
                }

                iDynTree::SolidShape *shape = convertGeometry(visual->Geom());
                if (shape)
                {
                    shape->setName(visual->Name());
                    shape->setLink_H_geometry(poseToTransform(visual->RawPose()));
                    m_model.visualSolidShapes().addSingleLinkSolidShape(addedLinkIndex,
                                                                        *shape);
                    delete shape;
                }
            }

            // Parse collision geometries
            for (uint64_t collisionIdx = 0; collisionIdx < sdfLink->CollisionCount();
                 ++collisionIdx)
            {
                const sdf::Collision *collision = sdfLink->CollisionByIndex(collisionIdx);
                if (!collision)
                {
                    std::string errMsg = "Failed to get collision from link " + sdfLink->Name();
                    reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                    return false;
                }

                if (!collision->Geom())
                {
                    std::string errMsg = "Collision " + collision->Name() + " in link " +
                                         sdfLink->Name() + " has no geometry";
                    reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                    return false;
                }

                iDynTree::SolidShape *shape = convertGeometry(collision->Geom());
                if (shape)
                {
                    shape->setName(collision->Name());
                    shape->setLink_H_geometry(poseToTransform(collision->RawPose()));
                    m_model.collisionSolidShapes().addSingleLinkSolidShape(addedLinkIndex,
                                                                           *shape);
                    delete shape;
                }
            }
        }

        // Add all joints to the model
        for (uint64_t jointIdx = 0; jointIdx < sdfModel->JointCount(); ++jointIdx)
        {
            const sdf::Joint *sdfJoint = sdfModel->JointByIndex(jointIdx);
            if (!sdfJoint)
            {
                reportError("SDFormatDocument", "convertSDFormatToModel",
                            "Failed to get joint from model");
                return false;
            }

            // Get parent and child links
            std::string parentLinkName = sdfJoint->ParentName();
            std::string childLinkName = sdfJoint->ChildName();

            LinkIndex parentLinkIndex = m_model.getLinkIndex(parentLinkName);
            LinkIndex childLinkIndex = m_model.getLinkIndex(childLinkName);

            if (parentLinkIndex == iDynTree::LINK_INVALID_INDEX ||
                childLinkIndex == iDynTree::LINK_INVALID_INDEX)
            {
                std::string errMsg =
                    "Joint " + sdfJoint->Name() + " references invalid links";
                reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                return false;
            }

            // Get joint pose relative to child link frame
            // In SDFormat, joint pose is specified relative to the child link
            iDynTree::Transform jointTransform =
                poseToTransform(sdfJoint->SemanticPose().RawPose());

            // Create joint based on type
            sdf::JointType jointType = sdfJoint->Type();

            if (jointType == sdf::JointType::REVOLUTE)
            {
                // Revolute joint
                iDynTree::RevoluteJoint *revJoint = new iDynTree::RevoluteJoint();

                const sdf::JointAxis *jointAxis = sdfJoint->Axis(0);
                const gz::math::Vector3d &axisXyz = jointAxis->Xyz();
                const std::string &expressedIn = jointAxis->XyzExpressedIn();

                iDynTree::Direction axisDir_childFrame;
                iDynTree::Position axisOrigin_childFrame;

                // Resolve the axis to the child link frame
                // The axis is a line in space with a direction and passing through a point.
                // In SDFormat, the axis direction is given by <xyz> in the frame specified by expressed_in.
                // The axis passes through the origin of the expressed_in frame.
                if (expressedIn.empty() || expressedIn == sdfJoint->Name())
                {
                    // Default case: axis is in joint frame
                    // The joint frame origin is at (0,0,0) in the child link frame
                    axisDir_childFrame = iDynTree::Direction(axisXyz.X(), axisXyz.Y(), axisXyz.Z());
                    axisOrigin_childFrame = iDynTree::Position(0.0, 0.0, 0.0);
                }
                else if (expressedIn == "__model__")
                {
                    // Axis is in model frame, so we need to transform to child link frame
                    // Get child link's pose in model frame
                    const sdf::Link *childLink = sdfModel->LinkByName(childLinkName);
                    if (!childLink)
                    {
                        std::string errMsg = "Failed to find child link '" + childLinkName + "' for joint '" + sdfJoint->Name() + "'";
                        reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                        return false;
                    }

                    gz::math::Pose3d childLink_H_model;
                    sdf::Errors errors = childLink->SemanticPose().Resolve(childLink_H_model, "__model__");
                    if (!errors.empty())
                    {
                        std::string errMsg = "Failed to resolve child link pose for joint '" + sdfJoint->Name() + "'";
                        reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                        return false;
                    }

                    // Transform axis direction from model frame to child link frame
                    gz::math::Vector3d axisInChildFrame = childLink_H_model.Rot().Inverse() * axisXyz;
                    axisDir_childFrame = iDynTree::Direction(axisInChildFrame.X(), axisInChildFrame.Y(), axisInChildFrame.Z());

                    // Transform axis origin: the axis passes through model frame origin (0,0,0)
                    // Position of model origin in child link frame is -childLink_H_model.Pos()
                    gz::math::Vector3d modelOriginInChildFrame = -(childLink_H_model.Rot().Inverse() * childLink_H_model.Pos());
                    axisOrigin_childFrame = iDynTree::Position(modelOriginInChildFrame.X(), modelOriginInChildFrame.Y(), modelOriginInChildFrame.Z());
                }
                else
                {
                    // Axis is expressed in a named frame (could be a link or explicit frame)
                    // First try to find it as a link
                    const sdf::Link *expressedInLink = sdfModel->LinkByName(expressedIn);
                    const sdf::Link *childLink = sdfModel->LinkByName(childLinkName);

                    if (!childLink)
                    {
                        std::string errMsg = "Failed to find child link '" + childLinkName + "' for joint '" + sdfJoint->Name() + "'";
                        reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                        return false;
                    }

                    gz::math::Pose3d childLink_H_expressedIn;

                    if (expressedInLink)
                    {
                        // Resolve expressed_in link pose relative to model
                        gz::math::Pose3d expressedInLink_H_model;
                        sdf::Errors errors = expressedInLink->SemanticPose().Resolve(expressedInLink_H_model, "__model__");
                        if (!errors.empty())
                        {
                            std::string errMsg = "Failed to resolve expressed_in link pose for joint '" + sdfJoint->Name() + "'";
                            reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                            return false;
                        }

                        gz::math::Pose3d childLink_H_model;
                        errors = childLink->SemanticPose().Resolve(childLink_H_model, "__model__");
                        if (!errors.empty())
                        {
                            std::string errMsg = "Failed to resolve child link pose for joint '" + sdfJoint->Name() + "'";
                            reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                            return false;
                        }

                        childLink_H_expressedIn = childLink_H_model * expressedInLink_H_model.Inverse();
                    }
                    else
                    {
                        // Try to find it as an explicit frame
                        const sdf::Frame *expressedInFrame = sdfModel->FrameByName(expressedIn);
                        if (expressedInFrame)
                        {
                            // Resolve frame pose relative to child link
                            sdf::Errors errors = childLink->SemanticPose().Resolve(childLink_H_expressedIn, expressedIn);
                            if (!errors.empty())
                            {
                                std::string errMsg = "Failed to resolve frame '" + expressedIn + "' relative to child link for joint '" + sdfJoint->Name() + "'";
                                reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                                return false;
                            }
                            // Need to invert because we want expressedIn -> childLink
                            childLink_H_expressedIn = childLink_H_expressedIn.Inverse();
                        }
                        else
                        {
                            std::string errMsg = "Axis expressed_in frame '" + expressedIn + "' not found as link or frame for joint '" + sdfJoint->Name() + "'";
                            reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                            return false;
                        }
                    }

                    // Transform axis direction from expressed_in frame to child link frame
                    gz::math::Vector3d axisInChildFrame = childLink_H_expressedIn.Rot() * axisXyz;
                    axisDir_childFrame = iDynTree::Direction(axisInChildFrame.X(), axisInChildFrame.Y(), axisInChildFrame.Z());

                    // Transform axis origin: the axis passes through expressed_in frame origin (0,0,0)
                    // Position of expressed_in origin in child link frame
                    gz::math::Vector3d expressedInOriginInChildFrame = childLink_H_expressedIn.Pos();
                    axisOrigin_childFrame = iDynTree::Position(expressedInOriginInChildFrame.X(), expressedInOriginInChildFrame.Y(), expressedInOriginInChildFrame.Z());
                }

                iDynTree::Axis idynAxis(axisDir_childFrame, axisOrigin_childFrame);

                revJoint->setAttachedLinks(parentLinkIndex, childLinkIndex);
                revJoint->setRestTransform(jointTransform);
                revJoint->setAxis(idynAxis, childLinkIndex, parentLinkIndex);

                // Set joint limits if available
                if (jointAxis)
                {
                    revJoint->enablePosLimits(true);
                    revJoint->setPosLimits(0, jointAxis->Lower(), jointAxis->Upper());
                }

                m_model.addJoint(sdfJoint->Name(), revJoint);
            }
            else if (jointType == sdf::JointType::PRISMATIC)
            {
                // Prismatic joint
                iDynTree::PrismaticJoint *prismJoint = new iDynTree::PrismaticJoint();

                const sdf::JointAxis *jointAxis = sdfJoint->Axis(0);
                const gz::math::Vector3d &axisXyz = jointAxis->Xyz();
                const std::string &expressedIn = jointAxis->XyzExpressedIn();

                iDynTree::Direction axisDir_childFrame;
                iDynTree::Position axisOrigin_childFrame;

                // Resolve the axis to the child link frame
                // The axis is a line in space with a direction and passing through a point.
                // In SDFormat, the axis direction is given by <xyz> in the frame specified by expressed_in.
                // The axis passes through the origin of the expressed_in frame.
                if (expressedIn.empty() || expressedIn == sdfJoint->Name())
                {
                    // Default case: axis is in joint frame
                    // The joint frame origin is at (0,0,0) in the child link frame
                    axisDir_childFrame = iDynTree::Direction(axisXyz.X(), axisXyz.Y(), axisXyz.Z());
                    axisOrigin_childFrame = iDynTree::Position(0.0, 0.0, 0.0);
                }
                else if (expressedIn == "__model__")
                {
                    // Axis is in model frame - need to transform to child link frame
                    const sdf::Link *childLink = sdfModel->LinkByName(childLinkName);
                    if (!childLink)
                    {
                        std::string errMsg = "Failed to find child link '" + childLinkName + "' for joint '" + sdfJoint->Name() + "'";
                        reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                        return false;
                    }

                    gz::math::Pose3d childLink_H_model;
                    sdf::Errors errors = childLink->SemanticPose().Resolve(childLink_H_model, "__model__");
                    if (!errors.empty())
                    {
                        std::string errMsg = "Failed to resolve child link pose for joint '" + sdfJoint->Name() + "'";
                        reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                        return false;
                    }

                    // Transform axis direction from model frame to child link frame
                    gz::math::Vector3d axisInChildFrame = childLink_H_model.Rot().Inverse() * axisXyz;
                    axisDir_childFrame = iDynTree::Direction(axisInChildFrame.X(), axisInChildFrame.Y(), axisInChildFrame.Z());

                    // Transform axis origin: the axis passes through model frame origin (0,0,0)
                    // Position of model origin in child link frame is -childLink_H_model.Pos()
                    gz::math::Vector3d modelOriginInChildFrame = -(childLink_H_model.Rot().Inverse() * childLink_H_model.Pos());
                    axisOrigin_childFrame = iDynTree::Position(modelOriginInChildFrame.X(), modelOriginInChildFrame.Y(), modelOriginInChildFrame.Z());
                }
                else
                {
                    // Axis is expressed in a named frame (could be a link or explicit frame)
                    const sdf::Link *expressedInLink = sdfModel->LinkByName(expressedIn);
                    const sdf::Link *childLink = sdfModel->LinkByName(childLinkName);

                    if (!childLink)
                    {
                        std::string errMsg = "Failed to find child link '" + childLinkName + "' for joint '" + sdfJoint->Name() + "'";
                        reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                        return false;
                    }

                    gz::math::Pose3d childLink_H_expressedIn;

                    if (expressedInLink)
                    {
                        // It's a link - resolve relative transform
                        gz::math::Pose3d expressedInLink_H_model;
                        sdf::Errors errors = expressedInLink->SemanticPose().Resolve(expressedInLink_H_model, "__model__");
                        if (!errors.empty())
                        {
                            std::string errMsg = "Failed to resolve expressed_in link pose for joint '" + sdfJoint->Name() + "'";
                            reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                            return false;
                        }

                        gz::math::Pose3d childLink_H_model;
                        errors = childLink->SemanticPose().Resolve(childLink_H_model, "__model__");
                        if (!errors.empty())
                        {
                            std::string errMsg = "Failed to resolve child link pose for joint '" + sdfJoint->Name() + "'";
                            reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                            return false;
                        }

                        childLink_H_expressedIn = childLink_H_model * expressedInLink_H_model.Inverse();
                    }
                    else
                    {
                        // Try to find it as an explicit frame
                        const sdf::Frame *expressedInFrame = sdfModel->FrameByName(expressedIn);
                        if (expressedInFrame)
                        {
                            // Resolve frame pose relative to child link
                            sdf::Errors errors = childLink->SemanticPose().Resolve(childLink_H_expressedIn, expressedIn);
                            if (!errors.empty())
                            {
                                std::string errMsg = "Failed to resolve frame '" + expressedIn + "' relative to child link for joint '" + sdfJoint->Name() + "'";
                                reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                                return false;
                            }
                            childLink_H_expressedIn = childLink_H_expressedIn.Inverse();
                        }
                        else
                        {
                            std::string errMsg = "Axis expressed_in frame '" + expressedIn + "' not found as link or frame for joint '" + sdfJoint->Name() + "'";
                            reportError("SDFormatDocument", "convertSDFormatToModel", errMsg.c_str());
                            return false;
                        }
                    }

                    // Transform axis direction from expressed_in frame to child link frame
                    gz::math::Vector3d axisInChildFrame = childLink_H_expressedIn.Rot() * axisXyz;
                    axisDir_childFrame = iDynTree::Direction(axisInChildFrame.X(), axisInChildFrame.Y(), axisInChildFrame.Z());

                    // Transform axis origin: the axis passes through expressed_in frame origin (0,0,0)
                    // Position of expressed_in origin in child link frame
                    gz::math::Vector3d expressedInOriginInChildFrame = childLink_H_expressedIn.Pos();
                    axisOrigin_childFrame = iDynTree::Position(expressedInOriginInChildFrame.X(), expressedInOriginInChildFrame.Y(), expressedInOriginInChildFrame.Z());
                }

                iDynTree::Axis idynAxis(axisDir_childFrame, axisOrigin_childFrame);

                prismJoint->setAttachedLinks(parentLinkIndex, childLinkIndex);
                prismJoint->setRestTransform(jointTransform);
                prismJoint->setAxis(idynAxis, childLinkIndex, parentLinkIndex);

                // Set joint limits if available
                if (jointAxis)
                {
                    prismJoint->enablePosLimits(true);
                    prismJoint->setPosLimits(0, jointAxis->Lower(), jointAxis->Upper());
                }

                m_model.addJoint(sdfJoint->Name(), prismJoint);
            }
            else if (jointType == sdf::JointType::FIXED)
            {
                // Fixed joint
                iDynTree::FixedJoint *fixedJoint = new iDynTree::FixedJoint();

                fixedJoint->setAttachedLinks(parentLinkIndex, childLinkIndex);
                fixedJoint->setRestTransform(jointTransform);

                m_model.addJoint(sdfJoint->Name(), fixedJoint);
            }
            else
            {
                // Unsupported joint type
                std::string errMsg =
                    "Joint type " + std::to_string(static_cast<int>(jointType)) +
                    " for joint " + sdfJoint->Name() + " is not yet supported";
                reportWarning("SDFormatDocument", "convertSDFormatToModel",
                              errMsg.c_str());
            }
        }

        // Find and set the default base link
        std::vector<std::string> candidateRootLinks;
        for (size_t linkIdx = 0; linkIdx < m_model.getNrOfLinks(); ++linkIdx)
        {
            std::string linkName = m_model.getLinkName(static_cast<LinkIndex>(linkIdx));
            bool isChild = false;

            // Check if this link is a child of any joint
            for (size_t jointIdx = 0; jointIdx < m_model.getNrOfJoints(); ++jointIdx)
            {
                IJointPtr joint = m_model.getJoint(static_cast<JointIndex>(jointIdx));
                if (joint->getSecondAttachedLink() == static_cast<LinkIndex>(linkIdx))
                {
                    isChild = true;
                    break;
                }
            }

            if (!isChild)
            {
                candidateRootLinks.push_back(linkName);
            }
        }

        if (candidateRootLinks.empty())
        {
            reportError("SDFormatDocument", "convertSDFormatToModel",
                        "No root link found in model");
            return false;
        }

        if (candidateRootLinks.size() > 1)
        {
            std::string msg = "Multiple root links found: ";
            for (const auto &name : candidateRootLinks)
            {
                msg += name + " ";
            }
            msg += ". Using first one as default base.";
            reportWarning("SDFormatDocument", "convertSDFormatToModel", msg.c_str());
        }

        // Set the first candidate as default base
        LinkIndex baseLink = m_model.getLinkIndex(candidateRootLinks[0]);
        m_model.setDefaultBaseLink(baseLink);

        return true;
#else
        reportError("SDFormatDocument", "convertSDFormatToModel",
                    "iDynTree was not compiled with SDFormat support");
        return false;
#endif
    }

}
