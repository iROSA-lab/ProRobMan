#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Pose, Quaternion, Point
from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SpawnModel
import random

cube_urdf="""
<?xml version="1.0" ?>
<robot name="%NAME%" xmlns:xacro="http://ros.org/wiki/xacro">
    <link name="%NAME%">
        <visual>
            <geometry>
              <mesh filename="package://ProRobMan/meshes/%NAME%.dae" scale='1 1 1'/>
            </geometry>
        </visual>

        <collision>
            <geometry>
              <box size="0.045 0.045 0.045" />
            </geometry>
        </collision>
        
        <inertial>
          <mass value="0.066" />
          <inertia ixx="0.0000221859" ixy="0.0" ixz="0.0" iyy="0.0000221859" iyz="0.0" izz="0.0000221859" />
        </inertial>
    </link>

</robot>
"""

rospy.init_node('spawn_cubes', anonymous=True)
Spawning = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
rospy.wait_for_service("gazebo/spawn_urdf_model")

def spawn(id, position, orientation):
  model_name='cube_{0}'.format(id)
  model_xml = cube_urdf.replace('%NAME%', model_name)
  cube_pose = Pose(Point(*position), Quaternion(*quaternion_from_euler(*orientation)))
  Spawning(model_name, model_xml, "", cube_pose, "world")
  rospy.loginfo("%s was spawned in Gazebo", model_name)

# the ranges for generating cubs
# table size is 0.6 x 0.75
table_xlim=[-0.15,0.2]
table_ylim=[-0.3, 0.3]
table_zlim=[0.1, 0.2]
# table surface pose
xpose=0.5
ypose=0
zpose=0.5
for i in range(28):
  position=[xpose + random.uniform(*table_xlim),
            ypose + random.uniform(*table_ylim),
            zpose + random.uniform(*table_zlim)
  ]
  orientation=[random.uniform(-0.5,0.5), random.uniform(-0.5,0.5), random.uniform(-0.5,0.5)]
  spawn(i, position, orientation)