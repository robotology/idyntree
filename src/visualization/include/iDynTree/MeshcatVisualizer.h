// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MESHCAT_VISUALIZER_H
#define IDYNTREE_MESHCAT_VISUALIZER_H

#include <memory>
#include <string>

#include <iDynTree/Transform.h>
#include <iDynTree/Span.h>
#include <iDynTree/MatrixView.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/Model.h>

namespace iDynTree
{

  /**
   *  MeshcatVisualizer is an iDynTree-based wrapper to the [meshcat-cpp](https://github.com/ami-iit/meshcat-cpp) visualizer.
   * \note Only meshes are supported and the color is taken from the iDynTree::Model
   */
  class MeshcatVisualizer
  {
  public:
    MeshcatVisualizer();
    ~MeshcatVisualizer();

    /**
     * Load a given model in the visualizer.
     * @param model the model that should be loaded.
     * @param modelName the name of the model used in the visualizer. Each model you add needs to have an unique name.
     * @return True in case of success false otherwise/
     * @warning Only meshes are supported. The support to the primary shapes needs to be added.
     */
    bool loadModel(const iDynTree::Model &model,
                   const std::string &modelName);

    /**
     * Set the state of an already existing model in the visualizer.
     * @param world_T_base pose of the base of the model.
     * @param jointPositions position of the joints.
     * @param modelName the name of the model specified in MeshcatVisualizer::loadModel(),
     * @return True in case of success false otherwise.
     */
    bool setModelState(const iDynTree::Transform &world_T_base,
                       const iDynTree::VectorDynSize &jointPositions,
                       const std::string &modelName);

    /**
     * Set the state of an already existing model in the visualizer.
     * @param world_T_base 4x4 matrix representing the homogeneous transformation,
     * @param jointPositions position of the joints,
     * @param modelName the name of the model specified in MeshcatVisualizer::loadModel().
     * @return True in case of success false otherwise.
     */
    bool setModelState(const iDynTree::MatrixView<const double> &world_T_base,
                       const iDynTree::Span<const double> &jointPositions,
                       const std::string &modelName);

    /**
     * Load a sphere mesh in the visualizer.
     * @param radius radius of the sphere.
     * @param color the color of the mesh.
     * @param color needs to be a vector of 4 double between 0 and 1 representing RGBA.
     * @param name the name of the sphere. Each geometry you add needs to have an unique name.
     * @return True in case of success false otherwise.
     */
    bool loadSphere(const double radius,
                    const iDynTree::Span<const double> &color,
                    const std::string &name);

    /**
     * Load a cylinder mesh in the visualizer.
     * @param radius radius of the cylinder.
     * @param height height of the cylinder.
     * @param color the color of the mesh.
     * @param color needs to be a vector of 4 double between 0 and 1 representing RGBA.
     * @param name the name of the cylinder. Each geometry you add needs to have an unique name.
     * @return True in case of success false otherwise.
     */
    bool loadCylinder(const double radius, const double height,
                      const iDynTree::Span<const double> &color,
                      const std::string &name);

    /**
     * Load a box mesh in the visualizer.
     * @param width width of the box.
     * @param depth depth of the box.
     * @param height height of the box.
     * @param color the color of the mesh.
     * @param color needs to be a vector of 4 double between 0 and 1 representing RGBA.
     * @param name the name of the box. Each geometry you add needs to have an unique name.
     * @return True in case of success false otherwise.
     */
    bool loadBox(const double width, const double depth, const double height,
                 const iDynTree::Span<const double> &color,
                 const std::string &name);

    /**
     * Load an ellipsoid mesh in the visualizer.
     * @param a a-axis of the ellipsoid.
     * @param b b-axis of the ellipsoid.
     * @param c c-axis of the ellipsoid.
     * @param color the color of the mesh.
     * @param color needs to be a vector of 4 double between 0 and 1 representing RGBA.
     * @param name the name of the ellipsoid. Each geometry you add needs to have an unique name.
     * @return True in case of success false otherwise.
     */
    bool loadEllipsoid(const double a, const double b, const double c,
                       const iDynTree::Span<const double> &color,
                       const std::string &name);

    /**
     * set the pose of a primitive geometry mesh in the visualizer.
     * @param world_T_geometry pose of the geometry.
     * @param geometryName the name of the geometry specified in MeshcatVisualizer::loadSphere() || MeshcatVisualizer::loadCylinder() || MeshcatVisualizer::loadBox() || MeshcatVisualizer::loadEllipsoid().
     * @return True in case of success false otherwise.
     * Implementations available: for iDynTree::Transform, for iDynTree::MatrixView.
     */
    bool setPrimitiveGeometryTransform(const iDynTree::Transform &world_T_geometry,
                                       const std::string &geometryName);

    /**
     * set the pose of a primitive geometry mesh in the visualizer.
     * @param world_T_geometry pose of the geometry.
     * @param geometryName the name of the geometry specified in MeshcatVisualizer::loadSphere() || MeshcatVisualizer::loadCylinder() || MeshcatVisualizer::loadBox() || MeshcatVisualizer::loadEllipsoid().
     * @return True in case of success false otherwise.
     * Implementations available: for iDynTree::Transform, for iDynTree::MatrixView.
     */
    bool setPrimitiveGeometryTransform(const iDynTree::MatrixView<const double> &world_T_geometry,
                                       const std::string &geometryName);
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
