/*
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <MeshcatCpp/Material.h>
#include <MeshcatCpp/Meshcat.h>
#include <MeshcatCpp/Shape.h>

#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/SolidShapes.h>
#include <iDynTree/MeshcatVisualizer.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/Traversal.h>

#include <memory>
#include <string_view>
#include <utility>

using namespace iDynTree;

struct MeshcatVisualizer::Impl
{
    ::MeshcatCpp::Meshcat meshcat;

    struct ModelData
    {
        iDynTree::Model model;
        iDynTree::Traversal traversal;
        iDynTree::LinkPositions linkPositions;
    };

    std::unordered_map<std::string, ModelData> storedModels;
    [[nodiscard]] inline bool modelExists(const std::string &modelName)
    {
        return this->storedModels.find(modelName) != this->storedModels.end();
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

    bool addModelGeometryToView(ModelData &data,
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

                const iDynTree::Transform transform = (world_H_frame * linkSolidShape->getLink_H_geometry());

                this->meshcat.set_object(viewerName, mesh, material);
                this->meshcat.set_transform(viewerName, transform.asHomogeneousTransform());
            }

            linkIndex++;
        }

        return true;
    }

    bool updateModelGeometry(ModelData &data,
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
                                    "Provided: " +  std::to_string(jointPositions.size());
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
    Impl::ModelData data;
    m_pimpl->storedModels[modelName].model = model;
    auto &storedModel = m_pimpl->storedModels[modelName];
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

    Impl::ModelData &storedModel = m_pimpl->storedModels[modelName];
    return m_pimpl->updateModelGeometry(storedModel, world_T_base, jointPositions, modelName);
}

bool MeshcatVisualizer::setModelState(const iDynTree::MatrixView<const double> &world_T_base,
                                      const iDynTree::Span<const double> &jointPositions,
                                      const std::string &modelName)
{
    if (world_T_base.rows() != world_T_base.cols() || world_T_base.rows() != 4)
    {
        const std::string msg = "world_T_base needs to be a 4x4 matrix. Provided a "
                              + std::to_string(world_T_base.rows()) + "x"
                              + std::to_string(world_T_base.cols()) + " matrix.";
        reportError("MeshcatVisualizer", "setModelState", msg.c_str());
        return false;
    }

    return this->setModelState(iDynTree::Transform(world_T_base),
                               iDynTree::VectorDynSize(jointPositions),
                               modelName);
}


void MeshcatVisualizer::join()
{
    m_pimpl->meshcat.join();
}
