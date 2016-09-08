# How to load Sensor information in iDynTree (Legacy KDL-based functions)

**Warning: this page describe how sensor information is loaded from URDF files using the legacy KDL-based function [`iDynTree::sensorsListFromURDF`](http://wiki.icub.org/codyco/dox/html/idyntree/html/namespaceiDynTree.html#a2886164ec0bdd0526a5bbdd1c9b1dacc). 
  For the URDF extensions used in the modern iDynTree-native code, please check [model_loading](model_loading.md)**. 
  
### Gazebo URDF Sensor extensions 
The old KDL-based joint torque estimation classes are loading information on the Six Axis F/T sensors present in the model using the Gazebo URDF extensions documented in [http://gazebosim.org/tutorials/?tut=ros_urdf](http://gazebosim.org/tutorials/?tut=ros_urdf). 

In particular an F/T sensor can be including with the following tags (for more info, check [`SDF` format `force_torque` sensors](http://sdformat.org/spec?ver=1.6&elem=sensor#force_torque_frame)). 

Example XML: 

~~~ 
<!-- Required: reference is referring to the joint to which the F/T sensor is attached -->
<gazebo reference="r_arm_ft_sensor">
        <!-- Required: name is the name to which the sensor is known, it can be equal or not to the joint name -->
        <sensor name="r_arm_ft_sensor" type="force_torque">
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
            <!-- Optional: if the frame option is set to "sensor", the orientation of the measuremerents 
                           is the one of the specified frame -->
            <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>
        </sensor>
</gazebo>
~~~
