from gazebo_ros import gazebo_interfae

cube_sdf="""
<?xml version='1.0'?>
<sdf version="1.4">
  <model name=%NAME%>
    <link name=%NAME%>
      <inertial>
        <mass>0.066</mass>
        <inertia> <!-- inertias are tricky to compute -->
          <!-- http://gazebosim.org/tutorials?tut=inertia&cat=build_robot -->
          <ixx>0.0000221859</ixx>       <!-- for a box: ixx = 0.083 * mass * (y*y + z*z) -->
          <ixy>0.0</ixy>         <!-- for a box: ixy = 0 -->
          <ixz>0.0</ixz>         <!-- for a box: ixz = 0 -->
          <iyy>0.0000221859</iyy>       <!-- for a box: iyy = 0.083 * mass * (x*x + z*z) -->
          <iyz>0.0</iyz>         <!-- for a box: iyz = 0 -->
          <izz>0.0000221859</izz>       <!-- for a box: izz = 0.083 * mass * (x*x + y*y) -->
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.045 0.045 0.045</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.045 0.045 0.045</size>
          </box>
        </geometry>
        <material>
            <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/%COLOR%</name>
            </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

# the cubes should be above the table
pose_z=0.5
pose_xlim=[0.2,0.8]
pose_ylim=[-0.35, 0.35]

counter=0
def spawn(color, positions, orientations):
    model_name=f'cube_{counter}'
    model_xml = cube_sdf \
        .replace('%NAME%', model_name) \
        .replace('%COLOR%', color)
    initial_pose = Pose(Point(*positions), Quaternion(*quaternion_from_euler(*orientations)))
    gazebo_interface.spawn_sdf_model_client(model_name, model_xml, rospy.get_namespace(),
                                            initial_pose, "", "/gazebo")
    rospy.loginfo("%s spawned in Gazebo as %s", model_name, gazebo_model_name)
    counter+=1
    return gazebo_model_name

colors=['RED, BLUE, GREEN']
rospy.init_node('spawn_cubes', anonymous=True)