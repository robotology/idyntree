// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <MeshcatCpp/Material.h>
#include <MeshcatCpp/Meshcat.h>
#include <MeshcatCpp/Shape.h>

#include <iDynTree/Utils.h>
#include <iDynTree/MatrixFixSize.h>
#include <iDynTree/Transform.h>
#include <iDynTree/VectorFixSize.h>
#include <iDynTree/Model.h>
#include <iDynTree/SolidShapes.h>
#include <iDynTree/MeshcatVisualizer.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/ForwardKinematics.h>
#include <iDynTree/LinkState.h>
#include <iDynTree/Traversal.h>

#include <memory>
#include <string_view>
#include <utility>
#include <unordered_map>
#include <unordered_set>

using namespace iDynTree;

struct MeshcatVisualizer::Impl
{
    ::MeshcatCpp::Meshcat meshcat;

    struct MultiBodyModelData
    {
        iDynTree::Model model;
        iDynTree::Traversal traversal;
        iDynTree::LinkPositions linkPositions;
    };
    std::unordered_map<std::string, MultiBodyModelData> storedMultiBodyModels;
    std::unordered_set<std::string> storedGeometries;

    [[nodiscard]] inline bool modelExists(const std::string &modelName)
    {
        return this->storedMultiBodyModels.find(modelName) != this->storedMultiBodyModels.end() ||
               this->storedGeometries.find(modelName) != this->storedGeometries.end();
    }

    [[nodiscard]] inline bool isMesh(const iDynTree::SolidShape &geometry)
    {
        if (!geometry.isExternalMesh())
        {
            return false;
        }

        std::string_view meshPath = geometry.asExternalMesh()->getFilename();

        if (meshPath.size() == 0)
        {
            return false;
        }

        std::size_t pos = meshPath.find_last_of(".");
        if (pos == std::string::npos)
        {
            return false;
        }
        const std::string_view format = meshPath.substr(pos + 1);
        return format == "dae" || format == "stl" || format == "obj";
    }

    // extact the path of the mesh from the meshcat tree
    std::string getMeshPathInMeshcatTree(const std::string &modelName,
                                         const std::string &linkName,
                                         const iDynTree::ExternalMesh &externalMesh)
    {
        const std::string fileName = externalMesh.getFileLocationOnLocalFileSystem();
        std::size_t pos_dot = fileName.find_last_of(".");
        std::size_t pos_slash = fileName.find_last_of("/");
        if (pos_slash == std::string::npos)
        {
            pos_slash = 0;
        }

        return modelName + "/" + linkName + "/" + fileName.substr(pos_slash + 1, pos_dot - pos_slash - 1);
    }

    bool addModelGeometryToView(MultiBodyModelData &data,
                                const std::string &modelName)
    {
        iDynTree::Model &model = data.model;
        iDynTree::Traversal &traversal = data.traversal;
        iDynTree::LinkPositions &linkPositions = data.linkPositions;

        iDynTree::VectorDynSize tmpVector(model.getNrOfJoints());
        tmpVector.zero();

        if (!iDynTree::ForwardPositionKinematics(model,
                                                 traversal,
                                                 iDynTree::Transform::Identity(),
                                                 tmpVector,
                                                 linkPositions))
        {
            const std::string msg = "Unable tos solve the inverse kinematics for the model named " + modelName;
            reportError("MeshcatVisualizer::Impl", "addModelGeometryToView", msg.c_str());
            return false;
        }

        const iDynTree::ModelSolidShapes &visualSolidShapes = model.visualSolidShapes();
        const auto &linksSolidShapes = visualSolidShapes.getLinkSolidShapes();

        int linkIndex = 0;
        for (const auto &linkSolidShapes : linksSolidShapes)
        {
            const iDynTree::Transform &world_H_frame = linkPositions(linkIndex);
            const std::string linkName = model.getLinkName(linkIndex);

            for (auto linkSolidShape : linkSolidShapes)
            {
                assert(linkSolidShape);

                if (!this->isMesh(*linkSolidShape))
                {
                    continue;
                }

                iDynTree::ExternalMesh *externalMesh = linkSolidShape->asExternalMesh();
                const std::string viewerName = this->getMeshPathInMeshcatTree(modelName, linkName, *externalMesh);

                MeshcatCpp::Mesh mesh(externalMesh->getFileLocationOnLocalFileSystem(),
                                      externalMesh->getScale()(0));
                MeshcatCpp::Material material = MeshcatCpp::Material::get_default_material();

                const iDynTree::Vector4 color = externalMesh->getMaterial().color();
                material.set_color(color(0) * 255, color(1) * 255, color(2) * 255);
                if (color(3) < 1)
                {
                    material.opacity = color(3);
                    material.transparent = true;
                }

                const iDynTree::Transform transform = (world_H_frame * linkSolidShape->getLink_H_geometry());

                this->meshcat.set_object(viewerName, mesh, material);
                this->meshcat.set_transform(viewerName, transform.asHomogeneousTransform());
            }

            linkIndex++;
        }

        return true;
    }

    bool updateModelGeometry(MultiBodyModelData &data,
                             const iDynTree::Transform &basePose,
                             const iDynTree::VectorDynSize &jointPositions,
                             const std::string &modelName)
    {

        const iDynTree::Model &model = data.model;
        const iDynTree::Traversal &traversal = data.traversal;
        iDynTree::LinkPositions &linkPositions = data.linkPositions;

        if (jointPositions.size() != model.getNrOfDOFs())
        {
            const std::string msg = "Wrong size of the jointPositions vector for the model " + modelName +
                                    ". Expected: " + std::to_string(model.getNrOfDOFs()) +
                                    "Provided: " + std::to_string(jointPositions.size());
            reportError("MeshcatVisualizer::Impl", "updateModelGeometry", msg.c_str());
            return false;
        }

        if (!iDynTree::ForwardPositionKinematics(
                model, traversal, basePose, jointPositions, linkPositions))
        {
            const std::string msg = "Unable to solve the inverse kinematics for the model named " + modelName;
            reportError("MeshcatVisualizer::Impl", "updateModelGeometry", msg.c_str());
            return false;
        }

        // update the visual shape
        const iDynTree::ModelSolidShapes &visualSolidShapes = model.visualSolidShapes();
        const auto &linksSolidShapes = visualSolidShapes.getLinkSolidShapes();

        int linkIndex = 0;
        for (const auto &linkSolidShapes : linksSolidShapes)
        {
            const iDynTree::Transform &world_H_frame = linkPositions(linkIndex);
            const std::string linkName = model.getLinkName(linkIndex);

            for (auto linkSolidShape : linkSolidShapes)
            {
                assert(linkSolidShape);

                if (!this->isMesh(*linkSolidShape))
                {
                    continue;
                }

                iDynTree::ExternalMesh *externalMesh = linkSolidShape->asExternalMesh();
                const std::string viewerName = this->getMeshPathInMeshcatTree(modelName, linkName, *externalMesh);

                const iDynTree::Transform transform = (world_H_frame * linkSolidShape->getLink_H_geometry());

                this->meshcat.set_transform(viewerName, transform.asHomogeneousTransform());
            }

            linkIndex++;
        }

        return true;
    }
};

MeshcatVisualizer::MeshcatVisualizer()
{
    m_pimpl = std::make_unique<MeshcatVisualizer::Impl>();
}

MeshcatVisualizer::~MeshcatVisualizer() = default;

bool MeshcatVisualizer::loadModel(const iDynTree::Model &model,
                                  const std::string &modelName)
{
    if (m_pimpl->modelExists(modelName))
    {
        const std::string msg = "The model named " + modelName + "already exists.";
        reportError("MeshcatVisualizer", "loadModel", msg.c_str());
        return false;
    }

    // add the model
    Impl::MultiBodyModelData data;
    m_pimpl->storedMultiBodyModels[modelName].model = model;
    auto &storedModel = m_pimpl->storedMultiBodyModels[modelName];
    storedModel.model.computeFullTreeTraversal(storedModel.traversal);
    storedModel.linkPositions.resize(storedModel.model);

    return m_pimpl->addModelGeometryToView(storedModel, modelName);
    ;
}

bool MeshcatVisualizer::setModelState(const iDynTree::Transform &world_T_base,
                                      const iDynTree::VectorDynSize &jointPositions,
                                      const std::string &modelName)
{
    if (!m_pimpl->modelExists(modelName))
    {
        const std::string msg = "Unable to find the model named " + modelName;
        reportError("MeshcatVisualizer", "setModelState", msg.c_str());
        return false;
    }

    Impl::MultiBodyModelData &storedModel = m_pimpl->storedMultiBodyModels[modelName];
    return m_pimpl->updateModelGeometry(storedModel, world_T_base, jointPositions, modelName);
}

bool MeshcatVisualizer::setModelState(const iDynTree::MatrixView<const double> &world_T_base,
                                      const iDynTree::Span<const double> &jointPositions,
                                      const std::string &modelName)
{
    if (world_T_base.rows() != world_T_base.cols() || world_T_base.rows() != 4)
    {
        const std::string msg = "world_T_base needs to be a 4x4 matrix. Provided a " + std::to_string(world_T_base.rows()) + "x" + std::to_string(world_T_base.cols()) + " matrix.";
        reportError("MeshcatVisualizer", "setModelState", msg.c_str());
        return false;
    }

    return this->setModelState(iDynTree::Transform(world_T_base),
                               iDynTree::VectorDynSize(jointPositions),
                               modelName);
}

bool MeshcatVisualizer::loadSphere(const double radius,
                                   const iDynTree::Span<const double> &color,
                                   const std::string &name)
{
    // check if the model already exists
    if (m_pimpl->modelExists(name))
    {
        const std::string msg = "The model named " + name + "already exists.";
        reportError("MeshcatVisualizer", "loadSphere", msg.c_str());
        return false;
    }

    // check if the size of the vector is equal to 4
    if (color.size() != 4)
    {
        const std::string msg = "The color needs to be a vector of 4 elements between 0 and 1. Provided: " + std::to_string(color.size());
        reportError("MeshcatVisualizer", "loadSphere", msg.c_str());
        return false;
    }

    // check if all the elements in color are between 0 and 1
    if (color(0) < 0 || color(0) > 1 ||
        color(1) < 0 || color(1) > 1 ||
        color(2) < 0 || color(2) > 1 ||
        color(3) < 0 || color(3) > 1)
    {
        const std::string msg = "The color needs to be a vector of 4 elements between 0 and 1. Provided: " + std::to_string(color[0]) + ", " + std::to_string(color[1]) + ", " + std::to_string(color[2]) + ", " + std::to_string(color[3]);
        reportError("MeshcatVisualizer", "loadSphere", msg.c_str());
        return false;
    }

    MeshcatCpp::Material m = MeshcatCpp::Material::get_default_material();
    m.set_color(uint8_t(color[0] * 255), uint8_t(color[1] * 255), uint8_t(color[2] * 255));
    if (color[3] < 1)
    {
        m.opacity = color[3];
        m.transparent = true;
    }

    m_pimpl->meshcat.set_object(name, MeshcatCpp::Sphere(radius), m);

    m_pimpl->storedGeometries.insert(name);

    return true;
}

bool MeshcatVisualizer::loadCylinder(const double radius, const double height,
                                     const iDynTree::Span<const double> &color,
                                     const std::string &name)
{
    // check if the model already exists
    if (m_pimpl->modelExists(name))
    {
        const std::string msg = "The model named " + name + "already exists.";
        reportError("MeshcatVisualizer", "loadCylinder", msg.c_str());
        return false;
    }

    // check if the size of the vector is equal to 4
    if (color.size() != 4)
    {
        const std::string msg = "The color needs to be a vector of 4 elements between 0 and 1. Provided: " + std::to_string(color.size());
        reportError("MeshcatVisualizer", "loadCylinder", msg.c_str());
        return false;
    }

    // check if all the elements in color are between 0 and 1
    if (color(0) < 0 || color(0) > 1 ||
        color(1) < 0 || color(1) > 1 ||
        color(2) < 0 || color(2) > 1 ||
        color(3) < 0 || color(3) > 1)
    {
        const std::string msg = "The color needs to be a vector of 4 elements between 0 and 1. Provided: " + std::to_string(color[0]) + ", " + std::to_string(color[1]) + ", " + std::to_string(color[2]) + ", " + std::to_string(color[3]);
        reportError("MeshcatVisualizer", "loadCylinder", msg.c_str());
        return false;
    }

    MeshcatCpp::Material m = MeshcatCpp::Material::get_default_material();
    m.set_color(uint8_t(color[0] * 255), uint8_t(color[1] * 255), uint8_t(color[2] * 255));
    if (color[3] < 1)
    {
        m.opacity = color[3];
        m.transparent = true;
    }

    m_pimpl->meshcat.set_object(name, MeshcatCpp::Cylinder(radius, height), m);

    m_pimpl->storedGeometries.insert(name);

    return true;
}

bool MeshcatVisualizer::loadBox(const double width, const double depth, const double height,
                                const iDynTree::Span<const double> &color,
                                const std::string &name)
{
    // check if the model already exists
    if (m_pimpl->modelExists(name))
    {
        const std::string msg = "The model named " + name + "already exists.";
        reportError("MeshcatVisualizer", "loadBox", msg.c_str());
        return false;
    }

    // check if the size of the vector is equal to 4
    if (color.size() != 4)
    {
        const std::string msg = "The color needs to be a vector of 4 elements between 0 and 1. Provided: " + std::to_string(color.size());
        reportError("MeshcatVisualizer", "loadBox", msg.c_str());
        return false;
    }

    // check if all the elements in color are between 0 and 1
    if (color(0) < 0 || color(0) > 1 ||
        color(1) < 0 || color(1) > 1 ||
        color(2) < 0 || color(2) > 1 ||
        color(3) < 0 || color(3) > 1)
    {
        const std::string msg = "The color needs to be a vector of 4 elements between 0 and 1. Provided: " + std::to_string(color[0]) + ", " + std::to_string(color[1]) + ", " + std::to_string(color[2]) + ", " + std::to_string(color[3]);
        reportError("MeshcatVisualizer", "loadBox", msg.c_str());
        return false;
    }

    MeshcatCpp::Material m = MeshcatCpp::Material::get_default_material();
    m.set_color(uint8_t(color[0] * 255), uint8_t(color[1] * 255), uint8_t(color[2] * 255));
    if (color[3] < 1)
    {
        m.opacity = color[3];
        m.transparent = true;
    }

    m_pimpl->meshcat.set_object(name, MeshcatCpp::Box(width, depth, height), m);

    m_pimpl->storedGeometries.insert(name);

    return true;
}

bool MeshcatVisualizer::loadEllipsoid(const double a, const double b, const double c,
                                      const iDynTree::Span<const double> &color,
                                      const std::string &name)
{
    // check if the model already exists
    if (m_pimpl->modelExists(name))
    {
        const std::string msg = "The model named " + name + "already exists.";
        reportError("MeshcatVisualizer", "loadEllipsoid", msg.c_str());
        return false;
    }

    // check if the size of the vector is equal to 4
    if (color.size() != 4)
    {
        const std::string msg = "The color needs to be a vector of 4 elements between 0 and 1. Provided: " + std::to_string(color.size());
        reportError("MeshcatVisualizer", "loadEllipsoid", msg.c_str());
        return false;
    }

    // check if all the elements in color are between 0 and 1
    if (color(0) < 0 || color(0) > 1 ||
        color(1) < 0 || color(1) > 1 ||
        color(2) < 0 || color(2) > 1 ||
        color(3) < 0 || color(3) > 1)
    {
        const std::string msg = "The color needs to be a vector of 4 elements between 0 and 1. Provided: " + std::to_string(color[0]) + ", " + std::to_string(color[1]) + ", " + std::to_string(color[2]) + ", " + std::to_string(color[3]);
        reportError("MeshcatVisualizer", "loadEllipsoid", msg.c_str());
        return false;
    }

    MeshcatCpp::Material m = MeshcatCpp::Material::get_default_material();
    m.set_color(uint8_t(color[0] * 255), uint8_t(color[1] * 255), uint8_t(color[2] * 255));
    if (color[3] < 1)
    {
        m.opacity = color[3];
        m.transparent = true;
    }

    m_pimpl->meshcat.set_object(name, MeshcatCpp::Ellipsoid(a, b, c), m);

    m_pimpl->storedGeometries.insert(name);

    return true;
}

bool MeshcatVisualizer::setPrimitiveGeometryTransform(const iDynTree::Transform &world_T_geometry,
                                                      const std::string &geometryName)
{
    return this->setPrimitiveGeometryTransform(iDynTree::make_matrix_view(world_T_geometry.asHomogeneousTransform()), geometryName);
}

bool MeshcatVisualizer::setPrimitiveGeometryTransform(const iDynTree::MatrixView<const double> &world_T_geometry,
                                                      const std::string &geometryName)
{
    if (world_T_geometry.rows() != world_T_geometry.cols() || world_T_geometry.rows() != 4)
    {
        const std::string msg = "world_T_geometry needs to be a 4x4 matrix. Provided a " + std::to_string(world_T_geometry.rows()) + "x" + std::to_string(world_T_geometry.cols()) + " matrix.";
        reportError("MeshcatVisualizer", "setPrimitiveGeometryTransform", msg.c_str());
        return false;
    }

    if (!m_pimpl->storedGeometries.count(geometryName))
    {
        const std::string msg = "Unable to find the geometry named " + geometryName;
        reportError("MeshcatVisualizer", "setPrimitiveGeometryTransform", msg.c_str());
        return false;
    }

    m_pimpl->meshcat.set_transform(geometryName, world_T_geometry);

    return true;
}

void MeshcatVisualizer::join()
{
    m_pimpl->meshcat.join();
}
