# How to load Models in iDynTree

Most of the classes and function related to Model loading/unloading are documented in the [`iDynTree ModelIO`](http://wiki.icub.org/codyco/dox/html/idyntree/html/group__iDynTreeModelIO.html) doxygen module.
In this document we will discuss the particular aspects of each file format supported by iDynTree for loading multibody systems information.

## URDF models
The main format used by iDynTree to load multibody models is the [URDF format](http://wiki.ros.org/urdf), originally developed in the ROS project.

iDynTree follows the [URDF specification](http://wiki.ros.org/urdf/XML/model) as much as possible.

Unfortunatly the URDF format semantics is not fully defined, so we try to match the semantics used by most used URDF parsers.

Furthermore, we also use some extension to the URDF specs, as defined in the following sections:

* [Spherical joints](#spherical-joints)
* [Sensors extensions](#sensor-extensions)

### Spherical joints

The `URDF` specification do not support Spherical Joints, that instead iDynTree support. To allow to easily load and export URDF models that contain spherical joints, by default iDynTree URDF parser detect if the model contains three consecutive `revolute` or `continuous` joints with this conditions:
* The three joint axis intersect at a single point
* The three joint axis are each one orthogonal to each other
* The two internal links have zero mass

If all these conditions are respected, the three revolute joints in the URDF model are substituted with a single iDynTree's SphericalJoint. Similarly, when a `iDynTree::Model` is exported, any `iDynTree::SphericalJoint` contained in it is exported as a three revolute joints that respect the aforementioned conditions.

To disable this behaviour that by default is enabled you can set the `ModelParserOptions::convertThreeRevoluteJointsToSphericalJoint` and `ModelExporterOptions::exportSphericalJointsAsThreeRevoluteJoints` options to `false`.

Note that this way of supporting spherical joints in URDF is compatible with the [`onshape-to-robot`'s `ball_to_euler` processor](https://onshape-to-robot.readthedocs.io/en/latest/processor_ball_to_euler.html).

### Sensor extensions
For loading sensor information, we support a non-standard extensions to the URDF format, documented in this section. Each sensor is encoded as a `<sensor>` xml element, that appears in the URDF file as a child of the root `<robot>` element.

* **name** *(required)* *(string)* A string (unique among the sensors in the URDF) that uniquely identifies the sensor.
* **type** *(required)* *(string)* A string that identifies the type of the sensors. Type supported by iDynTree : `force_torque` , `gyroscope`, `accelerometer`.

#### Joint sensors

##### `force_torque`
A sensor of type `force_torque` describes a a Six Axis Force Torque sensor, and it the only type of
sensor supported by iDynTree that is actually associated to an URDF joint. It is tipically associated
to a joint of type `fixed`, and its semantics closely match the one of the [`SDF` format `force_torque` sensors](http://sdformat.org/spec?ver=1.6&elem=sensor#force_torque_frame).
Example XML:
~~~
<sensor name="r_arm_ft_sensor" type="force_torque">
        <!-- Required: name of the joint to which the FT sensor is associated -->
        <parent joint="r_arm_ft_sensor"/>
        <!-- Requred: F/T specific attributes -->
        <force_torque>
                <!-- Required: Frame in which to report the wrench values.
                     Currently supported frames are:
                       "parent" report the wrench expressed in the orientation of the parent link frame,
                       "child" report the wrench expressed in the orientation of the child link frame,
                       "sensor" report the wrench expressed in the orientation of the sensor frame.
                     Note that for each option the point with respect to which the
                     torque component of the wrench is expressed is the child frame origin.
                -->
                <frame>child</frame>
                <!-- Required: Direction of the wrench measured by the sensor. The supported options are:
                       "parent_to_child" if the measured wrench is the one applied by parent link on the child link,
                        "child_to_parent" if the measured wrench is the one applied by the child link on the parent link.
                -->
                <measure_direction>child_to_parent</measure_direction>
        </force_torque>
        <!-- The sensor can optionally contains an <origin> element> if the <frame> element is set to sensor -->
</sensor>
~~~

#### Link sensors
All other sensors supported by iDynTree are of associated to a link, so their semantics is quite simple :
for each sensor we specify the name and the type (already specified with the attributes of the `<sensor>` tag) and
its position with respect to the link frame, contained in an origin element.

##### `gyroscope`
A `gyroscope` is a classical 3 axis Gyroscope, i.e. a sensor that measures the angular velocity of a frame.

Example XML:
~~~
 <sensor name="dummy_gyro" type="gyroscope">
        <parent link="link2"/>
        <origin xyz="0 -0.01 +0.01" rpy="0 -0 0"/>
</sensor>
~~~

##### `accelerometer`
A `accelerometer` is a classical 3 axis Accelerometer, i.e. a sensor that measures the proper acceleration of a frame.

Example XML:
~~~
 <sensor name="dummy_accelerometer" type="accelerometer">
        <parent link="link2"/>
        <origin xyz="0 -0.01 +0.01" rpy="0 -0 0"/>
</sensor>
~~~

## SDFormat models

iDynTree also supports loading robot models from the [SDFormat (Simulation Description Format)](http://sdformat.org/) specification, commonly used by Gazebo simulator. This support requires the `sdformat` library (version 16.x or later) and must be enabled at build time with the `IDYNTREE_USES_SDFORMAT` CMake option.

### Usage

The `ModelLoader` class automatically detects SDFormat files based on file extension (`.sdf` or `.world`):

```cpp
iDynTree::ModelLoader loader;
bool ok = loader.loadModelFromFile("robot.sdf");  // Auto-detected as SDFormat
```

You can also explicitly specify the format:

```cpp
bool ok = loader.loadModelFromFile("robot.sdf", "sdf");
```

### Supported Features

The SDFormat parser supports:
- **Links**: Full inertial properties (mass, center of mass, inertia tensor)
- **Joints**: Revolute, prismatic, and fixed joint types with limits
- **Sensors**: IMU sensors (creates both accelerometer and gyroscope), with proper transforms and link attachments
- **Visual Geometry**: Box, Sphere, Cylinder, and Mesh shapes with pose transforms
- **Collision Geometry**: Box, Sphere, Cylinder, and Mesh shapes with pose transforms
- **Multiple models**: Automatically extracts the first model from world files
- **Automatic base link detection**: Selects the root link (no parent joint) as the model base

### Sensor Support

The SDFormat parser supports IMU sensors attached to links:
- Each `<sensor type="imu">` creates both an `AccelerometerSensor` and a `GyroscopeSensor` in iDynTree
- Sensor names are automatically suffixed with `_acc` and `_gyro` respectively
- Sensor poses relative to the parent link are preserved

Example SDFormat sensor:
```xml
<link name="base_link">
  <sensor name="imu_sensor" type="imu">
    <pose>0 0 0.1 0 0 0</pose>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><stddev>0.009</stddev></noise></x>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><stddev>0.017</stddev></noise></x>
      </linear_acceleration>
    </imu>
  </sensor>
</link>
```

Accessing sensors after loading:
```cpp
iDynTree::ModelLoader loader;
loader.loadModelFromFile("robot.sdf");

const iDynTree::SensorsList& sensors = loader.model().sensors();
unsigned int nrAccel = sensors.getNrOfSensors(iDynTree::ACCELEROMETER);
unsigned int nrGyro = sensors.getNrOfSensors(iDynTree::GYROSCOPE);
```

### Geometry Support

The SDFormat parser supports visual and collision geometries for each link:
- Geometry types: Box, Sphere, Cylinder, and Mesh
- Pose transforms relative to the link frame are preserved
- Both visual and collision geometries are stored in the model

Example SDFormat geometries:
```xml
<link name="base_link">
  <visual name="visual_box">
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <box><size>0.1 0.1 0.2</size></box>
    </geometry>
  </visual>
  <collision name="collision_sphere">
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <sphere><radius>0.05</radius></sphere>
    </geometry>
  </collision>
</link>
```

Accessing geometries after loading:
```cpp
iDynTree::ModelLoader loader;
loader.loadModelFromFile("robot.sdf");

const iDynTree::Model& model = loader.model();
const iDynTree::ModelSolidShapes& visualShapes = model.visualSolidShapes();
const iDynTree::ModelSolidShapes& collisionShapes = model.collisionSolidShapes();

// Iterate through visual shapes for a specific link
iDynTree::LinkIndex linkIdx = model.getLinkIndex("base_link");
for (size_t i = 0; i < visualShapes.getLinkSolidShapes()[linkIdx].size(); i++) {
    const iDynTree::SolidShape* shape = visualShapes.getLinkSolidShapes()[linkIdx][i];
    // Use shape (Box, Sphere, Cylinder, or ExternalMesh)
}
```

### Current Limitations

The following SDFormat features are not yet supported:
- Force-torque sensors (typically attached to joints)
- Camera and other sensor types
- Additional joint types (ball, universal, screw)
- Nested model hierarchies
- Visual and material properties (colors, textures)
