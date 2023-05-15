/*
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_MESHCAT_VISUALIZER_H
#define IDYNTREE_MESHCAT_VISUALIZER_H

#include <memory>
#include <string>

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Model/Model.h>

namespace iDynTree
{

/**
 *  MeshcatVisualizer is a wrapper to the meshcat-cpp visualizer.
 */
class MeshcatVisualizer
{
public:
    MeshcatVisualizer();
    ~MeshcatVisualizer();

    /**
     * Load a given model in the visualizer
     * @param model the model that should be loaded.
     * @param modelName the name of the model used in the visualizer. Each model you add needs to have an unique name
     * @return True in case of success false otherwise
     * @warning Only meshes are supported. The support to the primary shapes needs to be added.
     */
    bool loadModel(const iDynTree::Model& model,
                   const std::string& modelName);

   /**
     * Set the state of an already existing model in the visualizer
     * @param basePose pose of the base of the model
     * @param jointPositions position of the joints
     * @param modelName the name of the model specified in MeshcatVisualizer::loadModel()
     * @return True in case of success false otherwise
     */
    bool setModelState(const iDynTree::Transform& basePose,
                       const iDynTree::VectorDynSize& jointPositions,
                       const std::string& modelName);

    /**
     * Utility function to make the meshcat interface run forever (until the user stops the
     * application)
     */
    void join();

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace iDynTree

#endif // IDYNTREE_MESHCAT_VISUALIZER_H
