# How to load Models in iDynTree

Most of the classes and function related to Model loading/unloading are documented in the [`iDynTree ModelIO`](http://wiki.icub.org/codyco/dox/html/idyntree/html/group__iDynTreeModelIO.html) doxygen module. 
In this document we will discuss the particular aspects of each file format supported by iDynTree for loading multibody systems information.

## URDF models 
The main format used by iDynTree to load multibody models is the [URDF format](http://wiki.ros.org/urdf), originally developed in the ROS project.

iDynTree follows the [URDF specification](http://wiki.ros.org/urdf/XML/model) as much as possible. 
Unfortunatly the URDF format semantics is not fully defined, so we try to match the semantics used by most widespared URDF parser. 

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






