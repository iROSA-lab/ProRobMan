# ProRobMan

To generate the cubes, you can install blender
```
bash install_blender.sh
```
Then run
```
./blender/blender --python scripts/generate_cubes.py
```
Data from the real-world was recorded into bag files <br>
To play the bag files with remapped topics (`/bag/points` for point cloud and `/bag/image_raw` for rgb images)
```
rosbag play -l bag_files/depth_registered_points.bag /xtion/depth_registered/points:=/bag/points
rosbag play -l bag_files/rgb.bag /xtion/rgb/image_raw:=/bag/image_raw
```