#include "idyntree_model.h"

#include "error_utilities.h"

#include <iDynTree/FixedJoint.h>
#include <iDynTree/IJoint.h>
#include <iDynTree/Indices.h>
#include <iDynTree/Link.h>
#include <iDynTree/Model.h>
#include <iDynTree/PrismaticJoint.h>
#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/SolidShapes.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/ModelTestUtils.h>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace iDynTree {
namespace bindings {
namespace {

using LinkShapes = std::vector<SolidShape*>;
using ModelShapes = std::vector<LinkShapes>;

// Wrappers around vectors (of vectors) of SolidShapes to simplify memory
// management with Python.
class ModelShapesWrapper {
 public:
  // View on the model shapes. Ownership remains to the caller.
  ModelShapes* shapes;
};

class LinkShapesWrapper {
 public:
  // View on the link shapes. Ownership of the pointed array remains to the
  // caller. SolidShape objects stored in the vector are owned by the vector.
  LinkShapes* shapes;
};

namespace py = ::pybind11;

void modelClassDefinition(py::class_<Model>& model) {
  model.def(py::init())
      .def("copy", py::overload_cast<>(&Model::copy, py::const_))
      .def("get_nr_of_links", &Model::getNrOfLinks)
      .def("get_link_index", &Model::getLinkIndex)
      .def("get_link_name", &Model::getLinkName)
      .def("get_link",
           py::overload_cast<const LinkIndex>(&Model::getLink, py::const_),
           py::return_value_policy::reference)
      .def("add_link", &Model::addLink)
      .def("get_nr_of_joints", &Model::getNrOfJoints)
      .def("get_joint_index", &Model::getJointIndex)
      .def("get_joint_name", &Model::getJointName)
      .def("get_joint",
           py::overload_cast<const JointIndex>(&Model::getJoint, py::const_),
           py::return_value_policy::reference)
      .def("add_joint", py::overload_cast<const std::string&, IJointConstPtr>(
                           &Model::addJoint))
      .def("add_additional_frame_to_link", &Model::addAdditionalFrameToLink)
      .def("get_nr_of_frames", &Model::getNrOfFrames)
      .def("get_frame_index", &Model::getFrameIndex)
      .def("get_frame_name", &Model::getFrameName)
      .def("get_frame_transform", &Model::getFrameTransform)
      .def("get_frame_link", &Model::getFrameLink)
      .def_property("default_base_link", &Model::getDefaultBaseLink, &Model::setDefaultBaseLink)
      .def("compute_full_tree_traversal", [](Model&m, const LinkIndex) -> std::unique_ptr<Traversal>{
        auto traversal = std::make_unique<Traversal>();
        m.computeFullTreeTraversal(*traversal);
        return traversal;
      })
      .def_property_readonly("visual_solid_shapes", 
                             py::overload_cast<>(&Model::visualSolidShapes))
      .def_property_readonly("collision_solid_shapes",
                             py::overload_cast<>(&Model::collisionSolidShapes))
      .def("__repr__", &Model::toString);
}

void linkClassDefinition(py::class_<Link>& link) {
  link.def(py::init())
      .def_property("inertia", &Link::getInertia, &Link::setInertia)
      .def("set_index", &Link::setIndex)
      .def("get_index", &Link::getIndex);
}

void jointClassDefinition(py::class_<IJoint>& joint) {
  joint.def("set_attached_links", &IJoint::setAttachedLinks)
      .def("set_rest_transform", &IJoint::setRestTransform)
      .def("get_rest_transform", &IJoint::getRestTransform)
      .def("get_first_attached_link", &IJoint::getFirstAttachedLink)
      .def("get_second_attached_link", &IJoint::getSecondAttachedLink)
      .def("enable_pos_limits", &IJoint::enablePosLimits)
      .def("has_pos_limits", &IJoint::hasPosLimits)
      .def("set_pos_limits", &IJoint::setPosLimits)
      .def("get_max_pos_limit", &IJoint::getMaxPosLimit)
      .def("get_min_pos_limit", &IJoint::getMinPosLimit)
      .def("get_joint_dynamics_type", &IJoint::getJointDynamicsType)
      .def("get_damping", &IJoint::getDamping)
      .def("get_static_friction", &IJoint::getStaticFriction)
      .def("set_joint_dynamics_type", &IJoint::setJointDynamicsType)
      .def("set_damping", &IJoint::setDamping)
      .def("set_static_friction", &IJoint::setStaticFriction);
}

void traversalClassDefinition(py::class_<Traversal>& traversal) {
  traversal.def(py::init())
      .def("get_nr_of_visited_links", &Traversal::getNrOfVisitedLinks)
      .def("get_link", &Traversal::getLink, py::return_value_policy::reference)
      .def("get_base_link", &Traversal::getBaseLink,
           py::return_value_policy::reference)
      .def("get_parent_link", &Traversal::getParentLink,
           py::return_value_policy::reference)
      .def("get_parent_joint", &Traversal::getParentJoint,
           py::return_value_policy::reference)
      .def("__repr__", &Traversal::toString);
}

    void utilityFunctionsDefinition(pybind11::module& module) {
        module
            .def("get_random_model",
                 iDynTree::getRandomModel,
                 py::arg("nr_of_joints"), py::arg("nr_of_additional_frames") = 10,
                 py::arg("only_revolute_joints") = false)
            .def("get_random_chain",
                 iDynTree::getRandomChain,
                 py::arg("nr_of_joints"), py::arg("nr_of_additional_frames") = 10,
                 py::arg("only_revolute_joints") = false);
    }

}  // namespace

void iDynTreeModelBindings(pybind11::module& module) {

  py::class_<Material>(module, "Material")
      .def(py::init())
      .def(py::init<const std::string&>())
      .def_property_readonly("name", &Material::name)
      .def_property("color", &Material::color, &Material::setColor)
      .def_property("texture", &Material::texture, &Material::setTexture);

  py::class_<SolidShape>(module, "_SolidShape")
      .def_property("name", &SolidShape::getName, &SolidShape::setName)
      .def_property("link_H_geometry", &SolidShape::getLink_H_geometry,
                    &SolidShape::setLink_H_geometry)
      .def_property("material", &SolidShape::getMaterial,
                    &SolidShape::setMaterial)
      .def("clone", &SolidShape::clone)
      .def("as_external_mesh", [](SolidShape& s) -> ExternalMesh*{
        auto* mesh = s.asExternalMesh();
        if (mesh) return mesh;
        raisePythonException(PyExc_ValueError,
                             "Specified object is not an ExternalMesh object.");
        return nullptr;
      }, py::return_value_policy::reference)
      .def("as_sphere", [](SolidShape& s) -> Sphere*{
        auto* mesh = s.asSphere();
        if (mesh) return mesh;
        raisePythonException(PyExc_ValueError,
                             "Specified object is not a Sphere object.");
        return nullptr;
      }, py::return_value_policy::reference);

  py::class_<ExternalMesh, SolidShape>(module, "ExternalMesh")
      .def(py::init())
      .def_property("filename", &ExternalMesh::getFilename,
                    &ExternalMesh::setFilename)
      .def_property("scale", &ExternalMesh::getScale, &ExternalMesh::setScale);

  py::class_<Sphere, SolidShape>(module, "Sphere")
      .def(py::init())
      .def_property("radius", &Sphere::getRadius, &Sphere::setRadius);

  py::class_<LinkShapesWrapper>(module, "LinkShapes")
    .def("__len__",
         [](const LinkShapesWrapper& v) { return v.shapes->size(); })
    .def(
        "__getitem__",
        [](const LinkShapesWrapper& v, std::size_t index) -> SolidShape* {
          return v.shapes->at(index);
        },
        py::return_value_policy::reference)
    .def("__setitem__",
         [](LinkShapesWrapper& v, std::size_t index, SolidShape* s) {
           SolidShape* old_shape = v.shapes->at(index);
           (*v.shapes)[index] = s->clone();
           delete old_shape;
         })
    .def("append", [](LinkShapesWrapper& v,
                      SolidShape* s) { v.shapes->push_back(s->clone()); })
    .def(
        "__iter__",
        [](LinkShapesWrapper& v) {
          return py::make_iterator(v.shapes->begin(), v.shapes->end());
        },
        py::keep_alive<0, 1>(), py::return_value_policy::reference)
    .def("clear", [](LinkShapesWrapper& v) {
      for (auto& shape : *v.shapes) {
        delete shape;
      }
      v.shapes->clear();
    });

  py::class_<ModelShapesWrapper>(module, "ModelShapes")
    .def("__len__",
         [](const ModelShapesWrapper& v) { return v.shapes->size(); })
    .def("__getitem__",
         [](const ModelShapesWrapper& v,
            std::size_t index) -> LinkShapesWrapper {
           LinkShapesWrapper wrapper;
           wrapper.shapes = &(*v.shapes)[index];
           return wrapper;
         });

  py::class_<ModelSolidShapes>(module, "ModelSolidShapes")
    .def(py::init())
    .def("clear", &ModelSolidShapes::clear)
    .def("__len__",
         [](const ModelSolidShapes& v) {
           return v.getLinkSolidShapes().size();
         })
    .def("resize", py::overload_cast<std::size_t>(&ModelSolidShapes::resize))
    .def("resize", py::overload_cast<const Model&>(&ModelSolidShapes::resize))
    .def(
        "__getitem__",
        [](ModelSolidShapes& shapes, std::size_t index) -> LinkShapesWrapper {
          LinkShapesWrapper wrapper;
          wrapper.shapes = &shapes.getLinkSolidShapes().at(index);
          return wrapper;
        })
    .def(
        "__getitem__",
        [](ModelSolidShapes& shapes,
           std::pair<std::size_t, std::size_t> indices) -> SolidShape* {
          return shapes.getLinkSolidShapes()
              .at(indices.first)
              .at(indices.second);
        },
        py::return_value_policy::reference)
    .def("__setitem__",
               [](ModelSolidShapes& shapes,
                  std::pair<std::size_t, std::size_t> indices, SolidShape* s) {
                 SolidShape* old_shape = shapes.getLinkSolidShapes()
                                             .at(indices.first)
                                             .at(indices.second);
                 shapes.getLinkSolidShapes()[indices.first][indices.second] =
                     s->clone();
                 delete old_shape;
               })
    .def_property_readonly(
        "solid_shapes", [](ModelSolidShapes& shapes) -> ModelShapesWrapper {
          ModelShapesWrapper wrapper;
          wrapper.shapes = &shapes.getLinkSolidShapes();
          return wrapper;
        });

  py::class_<Link> link(module, "Link");
  linkClassDefinition(link);

  py::class_<IJoint> ijoint(module, "_IJoint");
  jointClassDefinition(ijoint);

  py::class_<RevoluteJoint, IJoint>(module, "RevoluteJoint")
      .def(py::init())
      .def("set_axis",
           py::overload_cast<const Axis&, const LinkIndex, const LinkIndex>(
               &RevoluteJoint::setAxis));

  py::class_<PrismaticJoint, IJoint>(module, "PrismaticJoint")
      .def(py::init())
      .def("set_axis",
           py::overload_cast<const Axis&, const LinkIndex, const LinkIndex>(
               &PrismaticJoint::setAxis));

  py::class_<FixedJoint, IJoint>(module, "FixedJoint")
      .def(py::init());

  py::class_<Traversal> traversal(module, "Traversal");
  traversalClassDefinition(traversal);

  py::class_<Model> model(module, "Model");
  modelClassDefinition(model);

  utilityFunctionsDefinition(module);
}

}  // namespace bindings
}  // namespace iDynTree
